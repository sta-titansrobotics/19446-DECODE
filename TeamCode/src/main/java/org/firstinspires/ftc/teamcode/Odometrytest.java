package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@Autonomous(name = "Odometryyyyyy", group = "Concept??")
public class Odometrytest extends LinearOpMode {

    private DcMotorEx frontLeftMotor;
    private DcMotorEx frontRightMotor;
    private DcMotorEx backLeftMotor;
    private DcMotorEx backRightMotor;
    private DcMotorEx verticalLeftEncoder;
    private DcMotorEx verticalRightEncoder;
    private DcMotorEx horizontalEncoder; // You need a third encoder for 3-wheel odometry
    private BNO055IMU imu;

    private PositionTracker positionTracker;

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
        verticalLeftEncoder = hardwareMap.get(DcMotorEx.class, "leftOdo");
        verticalRightEncoder = hardwareMap.get(DcMotorEx.class, "rightOdo");
        horizontalEncoder = hardwareMap.get(DcMotorEx.class, "horizontalOdo"); // Map your third encoder here
        imu = hardwareMap.get(BNO055IMU.class, "imu"); // Make sure your config matches

        // Reset drive motor encoders for initialization
        frontLeftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        // Reset odometry wheel encoders for initialization
        verticalLeftEncoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        verticalRightEncoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        horizontalEncoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        // Set drive motors to RUN_USING_ENCODER mode
        frontLeftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Set odometry encoders to RUN_WITHOUT_ENCODER mode
        verticalLeftEncoder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        verticalRightEncoder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        horizontalEncoder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        // Setting PID coefficients
        PIDCoefficients pidNew = new PIDCoefficients(NEW_P, 0.0, NEW_D);
        frontLeftMotor.setPIDCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidNew);
        frontRightMotor.setPIDCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidNew);
        backLeftMotor.setPIDCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidNew);
        backRightMotor.setPIDCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidNew);

        positionTracker = new PositionTracker(verticalLeftEncoder, verticalRightEncoder, horizontalEncoder, imu);

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

            positionTracker.updatePosition();

            telemetry.addData("FL Pos", frontLeftMotor.getCurrentPosition());
            telemetry.addData("FR Pos", frontRightMotor.getCurrentPosition());
            telemetry.addData("BL Pos", backLeftMotor.getCurrentPosition());
            telemetry.addData("BR Pos", backRightMotor.getCurrentPosition());

            telemetry.addData("Odometry Left", verticalLeftEncoder.getCurrentPosition());
            telemetry.addData("Odometry Right", verticalRightEncoder.getCurrentPosition());
            telemetry.addData("Odometry Perpendicular", horizontalEncoder.getCurrentPosition());

            telemetry.addData("X Position", positionTracker.getX());
            telemetry.addData("Y Position", positionTracker.getY());
            telemetry.addData("Heading (Deg)", positionTracker.getHeading());
            telemetry.update();
        }

        // close motors
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
    }


    public static class PositionTracker {

        // You must calibrate these constants for your specific robot.
        // See the end of this file for instructions.
        public static final double TICKS_PER_INCH = 1550.0;
        public static final double TRACK_WIDTH = 13.7;
        public static final double PERPENDICULAR_X_OFFSET = 2.4;

        private DcMotorEx leftEncoder, rightEncoder, perpendicularEncoder;
        private BNO055IMU imu;

        // Robot's current pose (position and heading)
        private double x, y, heading;

        // Previous encoder readings
        private int prevLeftPos, prevRightPos, prevPerpPos;

        public PositionTracker(DcMotorEx left, DcMotorEx right, DcMotorEx perp, BNO055IMU imu) {
            this.leftEncoder = left;
            this.rightEncoder = right;
            this.perpendicularEncoder = perp;
            this.imu = imu;

            // Initialize position to (0,0) lets just say that
            this.x = 0.0;
            this.y = 0.0;
            this.heading = 0.0;

            // Initialize encoder readings
            this.prevLeftPos = leftEncoder.getCurrentPosition();
            this.prevRightPos = rightEncoder.getCurrentPosition();
            this.prevPerpPos = perpendicularEncoder.getCurrentPosition();

            // Initialize IMU
            BNO055IMU.Parameters imuParams = new BNO055IMU.Parameters();
            imuParams.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            this.imu.initialize(imuParams);
        }

        //update robot pose
        public void updatePosition() {
            int currentLeftPos = leftEncoder.getCurrentPosition();
            int currentRightPos = rightEncoder.getCurrentPosition();
            int currentPerpPos = perpendicularEncoder.getCurrentPosition();

            // Calculate delta encoder ticks
            int dLeft = currentLeftPos - prevLeftPos;
            int dRight = currentRightPos - prevRightPos;
            int dPerp = currentPerpPos - prevPerpPos;

            // Update previous encoder readings for the next cycle
            prevLeftPos = currentLeftPos;
            prevRightPos = currentRightPos;
            prevPerpPos = currentPerpPos;

            // Calculating da delta distances in inches??
            double dLeftInches = dLeft / TICKS_PER_INCH;
            double dRightInches = dRight / TICKS_PER_INCH;
            double dPerpInches = dPerp / TICKS_PER_INCH;

            // Calculate robot's change
            double dTheta = (dRightInches - dLeftInches) / TRACK_WIDTH;

            // Calculate robot's change in local X and Y
            double dX_local = dPerpInches - (dTheta * PERPENDICULAR_X_OFFSET);
            double dY_local = (dLeftInches + dRightInches) / 2.0;

            // Calculate robot's average heading
            double averageHeading = heading + (dTheta / 2.0);

            // Convert local change to global coordinates
            double dX_global = dX_local * Math.cos(averageHeading) - dY_local * Math.sin(averageHeading);
            double dY_global = dX_local * Math.sin(averageHeading) + dY_local * Math.cos(averageHeading);

            // Update robot's pose globally
            x += dX_global;
            y += dY_global;
            heading += dTheta;
        }

        public double getX() {
            return x;
        }


        public double getY() {
            return y;
        }

        public double getHeading() {
            // Returns the heading calculated from the odometry wheels.
            return Math.toDegrees(heading);
        }
    }
}
