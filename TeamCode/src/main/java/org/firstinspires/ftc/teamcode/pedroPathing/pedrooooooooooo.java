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
@Autonomous(name = "Visualizer Path Runner (Pedro 2.0+)", group = "Autonomous")
public class pedrooooooooooo extends LinearOpMode {

    // --- Define Runnables ---

    // Callback 1: Start flywheel, runs at 95% of Path1
    private Runnable startFlywheelRunnable = () -> {
        telemetry.addData("Path Callback", "Starting Flywheel PID");
        telemetry.update();
        flywheel.setTargetRPM(TARGET_RPM);
        // Motor is now running and correcting, robot is still moving
    };

    // Callback 2: Pause robot, shoot, and resume, runs at 100% of Path1
    private Runnable shootAndPauseRunnable = () -> {
        telemetry.addData("Path Callback", "Pausing Robot for Shoot Sequence");
        telemetry.update();

        // **1. Stop the drivetrain and wait for flywheel**
        // This is crucial: tell PedroPathing to hold position
        follower.pausePathFollowing();

        // Wait until the flywheel reaches target velocity
        while (opModeIsActive() && !flywheel.isReadyToShoot()) {
            flywheel.update(); // Update the flywheel's PID loop
            telemetry.addData("Flywheel", "Warming up: " + flywheel.getCurrentRPM());
            telemetry.update();
        }

        // **2. Shoot the balls**
        if (opModeIsActive()) {
            telemetry.addData("Flywheel", "Shooting!");
            telemetry.update();
            shootBalls(2000); // Wait 2000ms for balls to clear
        }

        // **3. Clean up and resume pathing**
        flywheel.stop(); // Stop the flywheel

        // This command tells the follower to continue on to the next path in the chain
        follower.resumePathFollowing();

        telemetry.addData("Path Callback", "Resuming Path");
        telemetry.update();
    };

    // --- Path Definitions from Visualizer ---

    /**
     * Stores and builds all the PathChain objects using the provided Follower.
     */
    public static class Paths {
        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;

        public Paths(Follower follower) {
            // Path 1: (56.0, 9.0) to (56.0, 14.0)
            Path1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(56.000, 9.000), new Pose(56.000, 14.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(109))
                    // 1. **Callback to START the flywheel motor at 95% completion**
                    .addParametricCallback(0.95, startFlywheelRunnable)

                    // 2. **Callback to PAUSE the robot at 100% completion**
                    .addParametricCallback(1.0, shootAndPauseRunnable)
                    .build();

            // Path 2: (56.0, 14.0) to (50.0, 35.0)
            Path2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(56.000, 14.000), new Pose(50.000, 35.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(109), Math.toRadians(180))
                    .build();

            // Path 3: (50.0, 35.0) to (9.0, 35.0)
            Path3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(50.000, 35.000), new Pose(9.000, 35.000))
                    )
                    .setTangentHeadingInterpolation() // Follows the curve's direction
                    .build();

            // Path 4: (9.0, 35.0) to (58.978, 13.946)
            Path4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(9.000, 35.000), new Pose(58.978, 13.946))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(109))
                    .build();
        }
    }

    // --- OpMode Execution Logic ---

    @Override
    public void runOpMode() throws InterruptedException {
        // 1. Initialize the Follower (UPDATED FOR PEDRO 2.0+)
        // The Follower is now created via a static method in your Constants file.
        Follower follower = Constants.createFollower(hardwareMap);

        // Optional: Set the initial pose if your path starts from a known location (like Path 1's start).
        // If your localization is working correctly, this step might be optional but is good practice.
        follower.setStartingPose(new Pose(56.000, 9.000, Math.toRadians(90)));

        // 2. Instantiate the Paths object, passing the initialized Follower
        Paths autonomousPaths = new Paths(follower);

        // Telemetry setup
        telemetry.addData("Status", "Initialization Complete");
        telemetry.addData("Start Pose", follower.getPose());
        telemetry.update();

        // Wait for the start button to be pressed
        waitForStart();

        // IMPORTANT: Update follower after starting to ensure localization is running
        follower.update();

        if (isStopRequested()) return;

        // --- EXECUTE PATHS SEQUENTIALLY ---

        telemetry.addData("Status", "Running Path 1...");
        telemetry.update();
        follower.followPath(autonomousPaths.Path1);

        // Check for stop request between paths
        if (isStopRequested()) return;

        telemetry.addData("Status", "Running Path 2...");
        telemetry.update();
        follower.followPath(autonomousPaths.Path2);

        if (isStopRequested()) return;

        telemetry.addData("Status", "Running Path 3...");
        telemetry.update();
        follower.followPath(autonomousPaths.Path3);

        if (isStopRequested()) return;

        telemetry.addData("Status", "Running Path 4...");
        telemetry.update();
        follower.followPath(autonomousPaths.Path4);

        // Final telemetry update
        telemetry.addData("Status", "Autonomous Routine Finished.");
        telemetry.addData("Final Pose", follower.getPose());
        telemetry.update();

        // Keep the OpMode running briefly so the user can see the final telemetry
        sleep(2000);
    }
}