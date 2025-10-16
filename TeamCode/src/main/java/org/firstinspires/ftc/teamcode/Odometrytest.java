package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;


@Autonomous(name="Concept: Change PID", group = "Concept")
public class Odometrytest extends LinearOpMode {
    private DcMotorEx frontLeftMotor;
    private DcMotorEx frontRightMotor;
    private DcMotorEx backLeftMotor;
    private DcMotorEx backRightMotor;

    public static final double NEW_P = 2.5;
    public static final double NEW_I = 0.1;
    public static final double NEW_D = 0.2;

    public void runOpMode() {
        // getting the motors
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "FL");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "FR");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "BL");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "BR");

        // wait for start command.
        waitForStart();

        PIDCoefficients pidOrig = frontLeftMotor.getPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        // change coefficients using methods included with DcMotorEx class.
        //have to check why PID Coefficients is crossing out ??
        PIDCoefficients pidNew = new PIDCoefficients(NEW_P, NEW_I, NEW_D);
        frontLeftMotor.setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidNew);

        // re-read coefficients and verify change according to tom.
        PIDCoefficients pidModified = frontLeftMotor.getPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        // display info to user at the telemetry
        while(opModeIsActive()) {
            telemetry.addData("Runtime", "%.03f", getRuntime());
            telemetry.addData("P,I,D (orig)", "%.04f, %.04f, %.0f",
                    pidOrig.p, pidOrig.i, pidOrig.d);
            telemetry.addData("P,I,D (modified)", "%.04f, %.04f, %.04f",
                    pidModified.p, pidModified.i, pidModified.d);
            telemetry.update();
        }
    }
}
