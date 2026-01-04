package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.util.TypeConversion;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.ArrayList;
import java.util.List;

@SuppressWarnings({"WeakerAccess", "unused"})
@I2cDeviceType
@DeviceProperties(name = "Grove Vision AI V2", description = "Seeed Studio Grove Vision AI V2", xmlTag = "GroveVisionAI")
public class LemonLight extends I2cDeviceSynchDevice<I2cDeviceSynch> {

    // Default I2C Address for Grove Vision AI V2 is 0x62
    public final static I2cAddr ADDRESS_I2C_DEFAULT = I2cAddr.create7bit(0x62);

    // Protocol Constants based on Seeed SSCMA
    // Note: The module can vary slightly depending on firmware version (SenseCraft vs default)
    // The standard register to read inference results often starts at 0x00 or via specific command headers.
    // For this driver, we assume a standard I2C read request which retrieves the inference buffer.
    private static final int READ_LENGTH_HEADER = 2; // Length of the size header

    public LemonLight(I2cDeviceSynch deviceClient, boolean deviceClientIsOwned) {
        super(deviceClient, deviceClientIsOwned);

        this.deviceClient.setI2cAddress(ADDRESS_I2C_DEFAULT);
        super.registerArmingStateCallback(false);
        this.deviceClient.engage();
    }

    /**
     * Represents a single object detected by the AI
     */
    public static class Recognition {
        public int x;
        public int y;
        public int width;
        public int height;
        public int confidence;
        public int labelId;

        public Recognition(int x, int y, int width, int height, int confidence, int labelId) {
            this.x = x;
            this.y = y;
            this.width = width;
            this.height = height;
            this.confidence = confidence;
            this.labelId = labelId;
        }

        @Override
        public String toString() {
            return String.format("ID:%d Conf:%d%% Box:[%d,%d,%d,%d]", labelId, confidence, x, y, width, height);
        }
    }
    public byte[] readRawBytes(int register, int count) {
        return deviceClient.read(register, count);
    }
    /**
     * Reads the latest detections from the module.
     * Note: This assumes the module is in "Continuous" inference mode or ready to serve data.
     */
    public List<Recognition> getDetections() {
        List<Recognition> detections = new ArrayList<>();

        // 1. Read the length of the available data first (2 bytes)
        // Many I2C AI modules send [Length LSB, Length MSB] first.
        byte[] lengthHeader = deviceClient.read(0x00, 2);

        if (lengthHeader.length < 2) return detections; // Read failed

        // Convert little-endian short
        short payloadLength = TypeConversion.byteArrayToShort(lengthHeader, ByteOrder.LITTLE_ENDIAN);

        if (payloadLength <= 0 || payloadLength > 1024) return detections; // Invalid or empty

        // 2. Read the full payload
        byte[] payload = deviceClient.read(0x00, payloadLength);

        // 3. Parse the Payload
        // The standard SSCMA format for object detection usually follows:
        // [Header] [Count] [Box1 X, Y, W, H, Score, Class] [Box2...]
        // Note: You may need to adjust offsets based on the exact model (YOLO vs MobileNet) firmware output.

        // This parser assumes a generic [x, y, w, h, score, class] 6-byte structure per object
        // You might need to adjust '6' to '8' depending on if values are shorts (2 bytes) or bytes.
        // Assuming 2 bytes per value (Shorts) for X, Y, W, H, Score, Class = 12 bytes per object.

        int bytesPerObject = 12; // Example: 6 fields * 2 bytes each
        int objectCount = payloadLength / bytesPerObject;

        ByteBuffer buffer = ByteBuffer.wrap(payload).order(ByteOrder.LITTLE_ENDIAN);

        for (int i = 0; i < objectCount; i++) {
            try {
                // Ensure we don't overrun buffer
                if (buffer.remaining() < bytesPerObject) break;

                // Example Parsing Logic - ADAPT THIS based on your specific model output
                short x = buffer.getShort();
                short y = buffer.getShort();
                short w = buffer.getShort();
                short h = buffer.getShort();
                short score = buffer.getShort();
                short target = buffer.getShort();

                detections.add(new Recognition(x, y, w, h, score, target));
            } catch (Exception e) {
                break;
            }
        }

        return detections;
    }

    @Override
    protected synchronized boolean doInitialize() {
        // Optional: Send a command to check ID or reset
        // 0x01 is often a config/sys register.
        // We just check if we can ping the device.
        return true;
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Other;
    }

    @Override
    public String getDeviceName() {
        return "Grove Vision AI V2";
    }
}
