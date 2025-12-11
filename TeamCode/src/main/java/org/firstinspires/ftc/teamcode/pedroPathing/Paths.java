package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.pedropathing.paths.PathChain; // Updated import location for PathChain
import com.pedropathing.geometry.BezierLine; // Updated import location for BezierLine
import com.pedropathing.geometry.Pose; // Updated import location for Pose
import com.pedropathing.follower.Follower; // Updated import location for Follower

/**
 * Defines the sequential paths for the autonomous routine.
 */
public class Paths {

    public PathChain Path1;
    public PathChain Path2;
    public PathChain Path3;
    public PathChain Path4;

    /**
     * Constructs all paths, including callbacks for the shooting sequence.
     * * @param follower The PedroPathing Follower instance.
     * @param startFlywheelRunnable Runnable to start the motor PID before the end of Path1.
     * @param shootAndPauseRunnable Runnable to pause the robot, execute the shoot sequence, and resume.
     */
    public Paths(Follower follower, Runnable startFlywheelRunnable, Runnable shootAndPauseRunnable) {

        // Path 1: Moves to the shooting position (56, 14).
        Path1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(56.000, 9.000), new Pose(56.000, 14.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(109))

                // 1. START FLYWHEEL at 95% completion (robot is still moving)
                .addParametricCallback(0.95, startFlywheelRunnable)

                // 2. PAUSE ROBOT and START SHOOT SEQUENCE at 100% completion
                .addParametricCallback(1.0, shootAndPauseRunnable)
                .build();

        // Path 2: Resumes after shooting.
        Path2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(56.000, 14.000), new Pose(50.000, 35.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(109), Math.toRadians(180))
                .build();

        // Path 3: Follows Path 2.
        Path3 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(50.000, 35.000), new Pose(9.000, 35.000))
                )
                .setTangentHeadingInterpolation()
                .build();

        // Path 4: Follows Path 3.
        Path4 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(9.000, 35.000), new Pose(58.978, 13.946))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(109))
                .build();
    }
}