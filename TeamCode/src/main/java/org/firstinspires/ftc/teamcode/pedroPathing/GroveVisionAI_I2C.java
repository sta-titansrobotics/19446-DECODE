package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.util.TypeConversion;

import java.util.ArrayList;
import java.util.List;

@I2cDeviceType
@DeviceProperties(name = "Grove Vision AI V2", xmlTag = "GroveVisionAI", description = "Grove Vision AI V2 Module")
public class GroveVisionAI_I2C extends I2cDeviceSynchDevice<I2cDeviceSynch> {

    public static final I2cAddr DEFAULT_I2C_ADDRESS = I2cAddr.create7bit(0x62);

    // Protocol constants based on reverse-engineered protocol
    private static final byte CMD_HEADER = 0x10;
    private static final byte CMD_INVOKE = 0x02;
    private static final byte CMD_READ_AVAILABLE = 0x03;
    private static final byte CMD_READ_DATA = 0x01;
    private static final int READ_BUFFER_SIZE = 250;

    private boolean deviceInitialized = false;
    private VisionResult latestResult = null;
    private long lastReadTimestamp = 0;
    private static final long MIN_READ_INTERVAL_MS = 50;

    public GroveVisionAI_I2C(I2cDeviceSynch deviceClient) {
        super(deviceClient, true);
        this.deviceClient.setI2cAddress(DEFAULT_I2C_ADDRESS);
        registerArmingStateCallback(false);
        this.deviceClient.engage();
    }

    @Override
    protected synchronized boolean doInitialize() {
        deviceInitialized = testConnection();
        return deviceInitialized;
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Other;
    }

    @Override
    public String getDeviceName() {
        return "Grove Vision AI V2";
    }

    public boolean isConnected() {
        return deviceInitialized;
    }

    private boolean testConnection() {
        try {
            byte[] testRead = deviceClient.read(0x00, 2);
            return testRead != null;
        } catch (Exception e) {
            return false;
        }
    }

    public boolean hasNewData() {
        return latestResult != null &&
                (System.currentTimeMillis() - lastReadTimestamp) < 500;
    }

    public synchronized VisionResult getLatestResult() {
        if (System.currentTimeMillis() - lastReadTimestamp < MIN_READ_INTERVAL_MS) {
            return latestResult;
        }

        try {
            int availableBytes = getAvailableBytes();
            if (availableBytes > 0) {
                byte[] rawData = readVisionData(availableBytes);
                latestResult = parseVisionResult(rawData);
                lastReadTimestamp = System.currentTimeMillis();
            }
        } catch (Exception e) {
            latestResult = null;
        }

        return latestResult;
    }

    private int getAvailableBytes() {
        try {
            byte[] cmd = new byte[]{CMD_HEADER, CMD_READ_AVAILABLE};
            deviceClient.write(0x00, cmd);

            Thread.sleep(5);

            byte[] response = deviceClient.read(0x00, 2);
            if (response != null && response.length >= 2) {
                return ((response[0] & 0xFF) << 8) | (response[1] & 0xFF);
            }
        } catch (Exception e) {
            // Suppress exception, return 0
        }
        return 0;
    }

    private byte[] readVisionData(int length) {
        try {
            int readLength = Math.min(length, READ_BUFFER_SIZE);
            byte[] cmd = new byte[]{
                    CMD_HEADER, CMD_READ_DATA,
                    (byte)((readLength >> 8) & 0xFF),
                    (byte)(readLength & 0xFF),
                    0x00, 0x00
            };
            deviceClient.write(0x00, cmd);

            Thread.sleep(10);

            return deviceClient.read(0x00, readLength);
        } catch (Exception e) {
            return new byte[0];
        }
    }

    private VisionResult parseVisionResult(byte[] data) {
        if (data == null || data.length < 10) {
            return new VisionResult();
        }

        String dataStr = new String(data);
        VisionResult result = new VisionResult();

        // Parse boxes format: [[x, y, w, h, score, target], ...]
        int boxesIdx = dataStr.indexOf("boxes:");
        if (boxesIdx >= 0) {
            try {
                String boxesSection = dataStr.substring(boxesIdx + 6);
                int endIdx = boxesSection.indexOf(']') + 1;
                if (endIdx > 0) {
                    String boxData = boxesSection.substring(0, endIdx);
                    result.boxes = parseBoxes(boxData);
                }
            } catch (Exception e) {
                // Parsing failed, return empty result
            }
        }

        return result;
    }

    private List<DetectionBox> parseBoxes(String boxData) {
        List<DetectionBox> boxes = new ArrayList<>();
        String cleaned = boxData.replace("[", "").replace("]", "");
        String[] values = cleaned.split(",");

        for (int i = 0; i + 5 < values.length; i += 6) {
            try {
                DetectionBox box = new DetectionBox();
                box.x = Integer.parseInt(values[i].trim());
                box.y = Integer.parseInt(values[i + 1].trim());
                box.width = Integer.parseInt(values[i + 2].trim());
                box.height = Integer.parseInt(values[i + 3].trim());
                box.confidence = Integer.parseInt(values[i + 4].trim());
                box.targetId = Integer.parseInt(values[i + 5].trim());
                boxes.add(box);
            } catch (NumberFormatException e) {
                // Skip malformed box
            }
        }

        return boxes;
    }

    public static class VisionResult {
        public List<DetectionBox> boxes = new ArrayList<>();

        public boolean hasDetections() {
            return !boxes.isEmpty();
        }

        public int getDetectionCount() {
            return boxes.size();
        }
    }

    public static class DetectionBox {
        public int x, y, width, height;
        public int confidence;
        public int targetId;

        public int getCenterX() {
            return x + width / 2;
        }

        public int getCenterY() {
            return y + height / 2;
        }
    }
}
