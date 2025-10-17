package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

@Autonomous(name = "Odometryyyyyy", group = "Concept")
public class Odometrytest extends LinearOpMode {

    private DcMotorEx frontLeftMotor;
    private DcMotorEx frontRightMotor;
    private DcMotorEx backLeftMotor;
    private DcMotorEx backRightMotor;

    // PID coefficients only P and d...
    public static final double NEW_P = 2.5;
    public static final double NEW_D = 0.2;

    // Target encoder ticks - just a hardcoded dummy
    public static final int TARGET_TICKS = 2000;

    @Override
    public void runOpMode() {
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "FL");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "FR");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "BL");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "BR");

        // Reset encoders - that's what google said idk if it works??
        frontLeftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // setting I as 0 cuz aint using it
        PIDCoefficients pidNew = new PIDCoefficients(NEW_P, 0.0, NEW_D);
        frontLeftMotor.setPIDCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidNew);
        frontRightMotor.setPIDCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidNew);
        backLeftMotor.setPIDCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidNew);
        backRightMotor.setPIDCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidNew);

        waitForStart();

        // Setting da target position
        frontLeftMotor.setTargetPosition(TARGET_TICKS);
        frontRightMotor.setTargetPosition(TARGET_TICKS);
        backLeftMotor.setTargetPosition(TARGET_TICKS);
        backRightMotor.setTargetPosition(TARGET_TICKS);

        frontLeftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        // Setting the powers of the motors cuz yes... should I set it to 1 OvO
        frontLeftMotor.setPower(0.5);
        frontRightMotor.setPower(0.5);
        backLeftMotor.setPower(0.5);
        backRightMotor.setPower(0.5);

        // telemetry stuff
        while (opModeIsActive() &&
                (frontLeftMotor.isBusy() || frontRightMotor.isBusy() ||
                        backLeftMotor.isBusy() || backRightMotor.isBusy())) {

            telemetry.addData("FL Pos", frontLeftMotor.getCurrentPosition());
            telemetry.addData("FR Pos", frontRightMotor.getCurrentPosition());
            telemetry.addData("BL Pos", backLeftMotor.getCurrentPosition());
            telemetry.addData("BR Pos", backRightMotor.getCurrentPosition());
            telemetry.update();
        }

        // close motors
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
    }
}
