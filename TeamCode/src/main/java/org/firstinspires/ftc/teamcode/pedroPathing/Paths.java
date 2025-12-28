package org.firstinspires.ftc.teamcode.pedroPathing;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.BezierCurve;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.pedropathing.paths.PathChain; // Updated import location for PathChain
import com.pedropathing.geometry.BezierLine; // Updated import location for BezierLine
import com.pedropathing.geometry.Pose; // Updated import location for Pose
import com.pedropathing.follower.Follower; // Updated import location for Follower

/**
 * Defines the sequential paths for the autonomous routine.
 */
@Config
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

        Path1 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(56.289, 9.242),
                                new Pose(57.130, 36.126),
                                new Pose(40.327, 35.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180))
                .build();

        Path2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(40.327, 35.000), new Pose(10.000, 35.000))
                )
                .setTangentHeadingInterpolation()
                .build();

        Path3 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(10.000, 35.000),
                                new Pose(74.100, 37.638),
                                new Pose(71.916, 71.580)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(127))
                .build();
    }
}