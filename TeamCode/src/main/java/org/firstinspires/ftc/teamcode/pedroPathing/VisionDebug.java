package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.pedroPathing.GroveVisionAI_I2C;

@TeleOp(name="Vision Test TeleOp", group="Vision")
public class VisionDebug extends OpMode {

    private GroveVisionAI_I2C groveVision;

    @Override
    public void init() {
        groveVision = hardwareMap.get(GroveVisionAI_I2C.class, "vision");

        telemetry.addData("Status", "Initializing Vision...");
        telemetry.update();

        if (groveVision.isConnected()) {
            telemetry.addData("Vision", "Ready");
        } else {
            telemetry.addData("Vision", "ERROR - Check I2C connection");
        }
        telemetry.update();
    }

    @Override
    public void loop() {
        telemetry.addData("Vision Connected", groveVision.isConnected());
        telemetry.addData("Has New Data", groveVision.hasNewData());

        GroveVisionAI_I2C.VisionResult result = groveVision.getLatestResult();

        if (result != null && result.hasDetections()) {
            telemetry.addData("Detection Count", result.getDetectionCount());

            for (int i = 0; i < result.boxes.size(); i++) {
                GroveVisionAI_I2C.DetectionBox box = result.boxes.get(i);
                telemetry.addData("Object " + i,
                        "ID=%d Conf=%d%% Pos=(%d,%d)",
                        box.targetId, box.confidence,
                        box.getCenterX(), box.getCenterY());
            }
        } else {
            telemetry.addData("Vision", "No objects detected");
        }

        telemetry.update();
    }
}
