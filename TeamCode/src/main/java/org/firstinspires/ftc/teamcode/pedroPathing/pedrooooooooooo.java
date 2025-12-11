package org.firstinspires.ftc.teamcode.pedroPathing; // Replace with your actual package name

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.pedropathing.paths.PathChain; // Updated import location for PathChain
import com.pedropathing.geometry.BezierLine; // Updated import location for BezierLine
import com.pedropathing.geometry.Pose; // Updated import location for Pose
import com.pedropathing.follower.Follower; // Updated import location for Follower
import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.ErrorCalculator;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.*;
import com.pedropathing.math.*;
import com.pedropathing.paths.*;
import com.pedropathing.telemetry.SelectableOpMode;
import com.pedropathing.util.*;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.bosch.BNO055IMU;
import java.lang.Math;
import com.acmerobotics.dashboard.FtcDashboard;
import java.util.List;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import com.qualcomm.robotcore.hardware.ColorSensor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import com.qualcomm.robotcore.hardware.LED;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

// IMPORTANT: This import MUST be changed to the location of your PedroPathing Constants file.
// Example: import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
// Replace this placeholder with your actual Constants class import.

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * Autonomous OpMode to run the predefined Pedropathing path set, compatible with
 * Pedropathing 2.0+ API.
 * This class assumes that the Pedropathing Constants and Tuning variables
 * are correctly set up in your project's Constants file.
 */
@Autonomous(name = "PedroPathing Shoot Sequence")
public class pedrooooooooooo extends LinearOpMode {

    // Subsystem instances
    private Follower follower;
    private FlywheelSubsystem flywheel;

    // Constants for the shoot
    private final double SHOOT_RPM = 1500; // Target RPM/Velocity
    private final double SHOOT_ANGLE_POS = 0.5; // Target angle for the shooter servo
    private final int NUM_BALLS = 2;

    DcMotor FL = hardwareMap.get(DcMotor.class, "FL"); // Expansion hub
    DcMotor BL = hardwareMap.get(DcMotor.class, "BL"); // Expansion hub
    DcMotor FR = hardwareMap.get(DcMotor.class, "FR"); // Expantion hub
    DcMotor BR = hardwareMap.get(DcMotor.class, "BR"); // Expantion hub

    DcMotor intake = hardwareMap.get(DcMotor.class, "intake");
    DcMotor elev = hardwareMap.get(DcMotor.class, "elev");
    DcMotor shoot = hardwareMap.get(DcMotor.class, "shoot");

    LED rled = hardwareMap.get(LED.class, "gled");
    LED gled = hardwareMap.get(LED.class, "rled");
    LED rled1 = hardwareMap.get(LED.class, "gled1");
    LED gled1 = hardwareMap.get(LED.class, "rled1");

    CRServo tubes = hardwareMap.get(CRServo.class, "tubes");
    Servo angles = hardwareMap.get(Servo.class, "shoot");

    ColorSensor sensorColor = hardwareMap.get(ColorSensor.class, "sensor_color");
    DistanceSensor sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_color");

    // --- Path Callbacks ---

    // 1. Starts the PID loop at 95% of Path1
    private Runnable startFlywheelRunnable = () -> {
        telemetry.addData("Callback", "Starting Flywheel PID (while moving)");
        telemetry.update();
        flywheel.setTargetRPM(SHOOT_RPM, SHOOT_ANGLE_POS);
    };

    // 2. Pauses, shoots, and resumes at 100% of Path1
    private Runnable shootAndPauseRunnable = () -> {
        telemetry.addLine("Reached shooting position. PAUSING...");
        telemetry.update();

        // 1. Stop the drivetrain
        follower.pausePathFollowing();

        // 2. Wait for PID to stabilize velocity
        while (opModeIsActive() && !flywheel.isReadyToShoot()) {
            flywheel.update(); // Keep PID running!
            telemetry.addData("Flywheel Status", "Warming up: " + flywheel.getCurrentRPM());
            telemetry.update();
            idle(); // Yield the thread to allow system updates
        }

        // 3. Shoot all balls
        for (int i = 0; i < NUM_BALLS; i++) {
            if (!opModeIsActive()) break;

            flywheel.update(); // Keep PID running during the shot
            tubes.setPower(1.0); // Actuate the shooting mechanism (e.g., pusher servo)
            telemetry.addData("Shooter", "Firing ball " + (i + 1));
            telemetry.update();
            sleep(500); // Wait for the ball to cycle (Adjust this time)
            tubes.setPower(0.0); // Stop the pusher mechanism
            sleep(200); // Brief pause before the next ball
        }

        // 4. Clean up and Resume
        flywheel.stop(); // Turn off the motor
        follower.resumePathFollowing(); // Tell PedroPathing to continue to Path2

        telemetry.addLine("Shooting complete. RESUMING path.");
        telemetry.update();
    };


    @Override
    public void runOpMode() throws InterruptedException {
        // --- Initialization ---

        // Initialize Flywheel Subsystem
        flywheel = new FlywheelSubsystem(hardwareMap);

        // Initialize Follower (assuming your drive train motors are configured inside the Follower)
        follower = Constants.createFollower(hardwareMap);

        // Initialize the shooting mechanism (CRServo or motor that pushes the ball)
        tubes = hardwareMap.get(CRServo.class, "tubes");

        // Initialize Paths with the defined runnables
        Paths paths = new Paths(follower, startFlywheelRunnable, shootAndPauseRunnable);

        // --- Start ---

        waitForStart();

        if (isStopRequested()) return;

        // Execute Path 1. The callbacks will automatically handle the pause/shoot/resume.
        follower.followPath(paths.Path1);

        // This loop keeps the OpMode active while the robot finishes Path 2, 3, and 4
        while (opModeIsActive() && follower.isBusy()) {
            telemetry.addData("Path Status", "Following Path...");
            telemetry.update();
            idle();
        }
    }
}


