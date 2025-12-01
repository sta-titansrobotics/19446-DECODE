package org.firstinspires.ftc.roadrunner;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import android.graphics.Color;

@TeleOp(name = "ColorSensorTest", group = "Sensor")
public class ColorSensor extends LinearOpMode {

    private NormalizedColorSensor colorSensor;

    @Override
    public void runOpMode() {

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");


        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight)colorSensor).enableLight(true);
        }


        if (colorSensor instanceof RevColorSensorV3) {
            ((RevColorSensorV3)colorSensor).setGain(10);
        }

        waitForStart();

        // Loop until the end of the match
        while (opModeIsActive()) {

            NormalizedRGBA colors = colorSensor.getNormalizedColors();

            final float[] hsvValues = new float[3];
            Color.colorToHSV(colors.toColor(), hsvValues);

            float hue = hsvValues[0];

            double greenMin = 150;
            double greenMax = 158;
            double purpleMin = 180;
            double purpleMax = 320;

            String detectedColor = "Unknown";
            if (hue >= greenMin && hue <= greenMax) {
                detectedColor = "Green";
            } else if (hue >= purpleMin && hue <= purpleMax) {
                detectedColor = "Purple";
            }

            telemetry.addData("Detected Color", detectedColor);
            telemetry.addData("Hue", "%.3f", hue);
            telemetry.update();
        }
    }
}
