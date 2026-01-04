package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.TypeConversion;

import java.nio.ByteOrder;
import java.util.List;

@TeleOp(name = "Vision AI Debugger", group = "Calibration")
public class VisionDebug extends OpMode {

    private LemonLight visionSensor;

    @Override
    public void init() {
        // Initialize the sensor using the hardware map name "vision_ai"
        visionSensor = hardwareMap.get(LemonLight.class, "vision_ai");

        telemetry.addData("Status", "Initialized");
        telemetry.addData("I2C Address", "0x62");
    }

    @Override
    public void loop() {
        // 1. RAW DATA DEBUGGING
        try {
            // FIX 1: Use the new public method we added to the driver
            byte[] lengthHeader = visionSensor.readRawBytes(0x00, 2);

            short payloadLength = TypeConversion.byteArrayToShort(lengthHeader, ByteOrder.LITTLE_ENDIAN);

            telemetry.addData("Payload Length", payloadLength);

            if (payloadLength > 0 && payloadLength < 200) {
                // Read the actual data
                byte[] payload = visionSensor.readRawBytes(0x00, payloadLength);

                // FIX 2: Use our manual bytesToHex method instead of TypeConversion
                telemetry.addData("Raw Hex", bytesToHex(payload));
            }

        } catch (Exception e) {
            telemetry.addData("Error", "I2C Read Failed: " + e.getMessage());
        }

        // 2. DRIVER TEST
        // Once Raw Data looks good, test the parsed output
        List<LemonLight.Recognition> detections = visionSensor.getDetections();

        telemetry.addData("Objects Detected", detections.size());

        for (int i = 0; i < detections.size(); i++) {
            LemonLight.Recognition r = detections.get(i);
            telemetry.addData("Obj " + i, r.toString());
        }

        telemetry.update();
    }

    // HELPER: Manually converts byte array to Hex String (e.g., "0A 0B")
    private String bytesToHex(byte[] bytes) {
        StringBuilder sb = new StringBuilder();
        for (byte b : bytes) {
            sb.append(String.format("%02X ", b));
        }
        return sb.toString();
    }
}