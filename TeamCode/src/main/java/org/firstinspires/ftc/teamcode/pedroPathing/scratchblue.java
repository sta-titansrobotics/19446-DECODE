package org.firstinspires.ftc.teamcode.pedroPathing; // make sure this aligns with class location

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.PoseHistory;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "scratch blue", group = "pedropathing")
public class scratchblue extends OpMode {
    public Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    static TelemetryManager telemetryM;

    // --- Pose Definitions (Organized at the beginning) ---

    private final Pose startPose = new Pose(39, 120, Math.toRadians(90));
    private final Pose pose1     = new Pose(43.4, 99.1,  Math.toRadians(127));
    private final Pose pose2     = new Pose(53.1, 83.5,  Math.toRadians(180));
    private final Pose pose3     = new Pose(18.5, 83.5,  Math.toRadians(180));
    private final Pose pose4     = new Pose(52.3, 92,  Math.toRadians(134));
    private final Pose pose5     = new Pose(51.4, 58.5,  Math.toRadians(180));
    private final Pose pose6     = new Pose(13, 58.5,  Math.toRadians(180));
    private final Pose pose7     = new Pose(56.8, 80.3,  Math.toRadians(133));
    private final Pose pose8     = new Pose(48.1, 36.5,  Math.toRadians(180));
    private final Pose pose9     = new Pose(13, 36.5,  Math.toRadians(180));
    private final Pose pose10    = new Pose(54.8, 89.4,  Math.toRadians(135));
    private final Pose pose11    = new Pose(44, 135, Math.toRadians(0));

    private double normalspeed = 0.9;
    private double intakespeed = 0.4;

/*
    private final Pose startPose = scratchtuning.startPose;
    private final Pose pose1 = scratchtuning.pose1;
    private final Pose pose2 = scratchtuning.pose2;
    private final Pose pose3 = scratchtuning.pose3;
    private final Pose pose4 = scratchtuning.pose4;
    private final Pose pose5 = scratchtuning.pose5;
    private final Pose pose6 = scratchtuning.pose6;
    private final Pose pose7 = scratchtuning.pose7;
    private final Pose pose8 = scratchtuning.pose8;
    private final Pose pose9 = scratchtuning.pose9;
    private final Pose pose10 = scratchtuning.pose10;
    private final Pose pose11 = scratchtuning.pose11;

 */



    private Path startpath;
    private PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9, Path10, Path11;

    private FlywheelSubsystem flywheel;
    private DcMotor tubes;
    private DcMotor elev;
    private DcMotor intake;
    private CRServo tubes1;

    public void buildPaths() {
        // Start Path
        startpath = new Path(new BezierLine(startPose, pose1));
        startpath.setLinearHeadingInterpolation(startPose.getHeading(), pose1.getHeading());

        // Path 2
        Path2 = follower.pathBuilder()
                .addPath(new BezierLine(pose1, pose2))
                .setLinearHeadingInterpolation(pose1.getHeading(), pose2.getHeading())
                .build();

        // Path 3 (Constant Heading)
        Path3 = follower.pathBuilder()
                .addPath(new BezierLine(pose2, pose3))
                .setConstantHeadingInterpolation(pose3.getHeading())
                .build();

        // Path 4
        Path4 = follower.pathBuilder()
                .addPath(new BezierLine(pose3, pose4))
                .setLinearHeadingInterpolation(pose3.getHeading(), pose4.getHeading())
                .build();

        // Path 5
        Path5 = follower.pathBuilder()
                .addPath(new BezierLine(pose4, pose5))
                .setLinearHeadingInterpolation(pose4.getHeading(), pose5.getHeading())
                .build();

        // Path 6 (Constant Heading)
        Path6 = follower.pathBuilder()
                .addPath(new BezierLine(pose5, pose6))
                .setConstantHeadingInterpolation(pose6.getHeading())
                .build();

        // Path 7
        Path7 = follower.pathBuilder()
                .addPath(new BezierLine(pose6, pose7))
                .setLinearHeadingInterpolation(pose6.getHeading(), pose7.getHeading())
                .build();

        // Path 8
        Path8 = follower.pathBuilder()
                .addPath(new BezierLine(pose7, pose8))
                .setLinearHeadingInterpolation(pose7.getHeading(), pose8.getHeading())
                .build();

        // Path 9 (Constant Heading)
        Path9 = follower.pathBuilder()
                .addPath(new BezierLine(pose8, pose9))
                .setConstantHeadingInterpolation(pose9.getHeading())
                .build();

        // Path 10
        Path10 = follower.pathBuilder()
                .addPath(new BezierLine(pose9, pose10))
                .setLinearHeadingInterpolation(pose9.getHeading(), pose10.getHeading())
                .build();

        // Path 11
        Path11 = follower.pathBuilder()
                .addPath(new BezierLine(pose10, pose11))
                .setLinearHeadingInterpolation(pose10.getHeading(), pose11.getHeading())
                .build();
    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.setMaxPower(normalspeed);
                follower.followPath(startpath);
                setPathState(1);
                break;
            case 1:

            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */
                if (follower.getCurrentTValue() > 0.75) {
                    flywheel.setTargetRPM(1225, 0.51); // Start flywheel at 95% of path
                }

                if(!follower.isBusy()) {
                    setPathState(100); // Transition to a custom "Shooting" state
                }

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */

                break;
            case 100:
                follower.holdPoint(pose1); // Keep robot still

                if (flywheel.isReadyToShoot()) {
                    tubes.setPower(1); // Turn on transfer
                    tubes1.setPower(-1);
                    elev.setPower(1);
                }

                if (pathTimer.getElapsedTimeSeconds() > 3) {
                    tubes.setPower(0);
                    tubes1.setPower(0);
                    elev.setPower(0);
                    flywheel.setTargetRPM(0, 0);
                    follower.followPath(Path2, true);
                    setPathState(2); // Resume normal path sequence
                }
                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.setMaxPower(intakespeed);
                    follower.followPath(Path3,true);
                    setPathState(3);
                }
                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */

                if(follower.isBusy()){
                    tubes.setPower(1); // Turn on transfer
                    tubes1.setPower(-1);
                    elev.setPower(-1);
                    intake.setPower(1);
                }

                if(!follower.isBusy()) {
                    /* Score Sample */
                    tubes.setPower(0); // Turn on transfer
                    tubes1.setPower(0);
                    elev.setPower(0);
                    intake.setPower(0);

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.setMaxPower(normalspeed);
                    follower.followPath(Path4,true);
                    setPathState(4);
                }
                break;
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */

                if (follower.getCurrentTValue() > 0.75) {
                    flywheel.setTargetRPM(1225, 0.65); // Start flywheel at 95% of path
                }

                if(!follower.isBusy()) {
                    setPathState(200); // Transition to a custom "Shooting" state
                }
                break;
            case 200:
                follower.holdPoint(pose4); // Keep robot still

                if (flywheel.isReadyToShoot()) {
                    tubes.setPower(1); // Turn on transfer
                    tubes1.setPower(-1);
                    elev.setPower(1);
                    intake.setPower(1);
                }

                if (pathTimer.getElapsedTimeSeconds() > 3) {
                    tubes.setPower(0);
                    tubes1.setPower(0);
                    elev.setPower(0);
                    intake.setPower(0);
                    flywheel.setTargetRPM(0, 0);
                    follower.followPath(Path5, true);
                    setPathState(5); // Resume normal path sequence
                }
                break;
            case 5:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.setMaxPower(intakespeed);
                    follower.followPath(Path6,true);
                    setPathState(6);
                }
                break;
            case 6:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(follower.isBusy()){
                    tubes.setPower(1); // Turn on transfer
                    tubes1.setPower(-1);
                    elev.setPower(-1);
                    intake.setPower(1);
                }

                if(!follower.isBusy()) {
                    /* Grab Sample */
                    tubes.setPower(0); // Turn on transfer
                    tubes1.setPower(0);
                    elev.setPower(0);
                    intake.setPower(0);
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.setMaxPower(normalspeed);
                    follower.followPath(Path7,true);
                    setPathState(7);
                }
                break;
            case 7:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if (follower.getCurrentTValue() > 0.75) {
                    flywheel.setTargetRPM(1225, 0.53); // Start flywheel at 95% of path
                }

                if(!follower.isBusy()) {
                    setPathState(300); // Transition to a custom "Shooting" state
                }
                break;
            case 300:
                follower.holdPoint(pose7); // Keep robot still

                if (flywheel.isReadyToShoot()) {
                    tubes.setPower(1); // Turn on transfer
                    tubes1.setPower(-1);
                    elev.setPower(1);
                    intake.setPower(1);
                }

                if (pathTimer.getElapsedTimeSeconds() > 3) {
                    tubes.setPower(0);
                    tubes1.setPower(0);
                    elev.setPower(0);
                    intake.setPower(0);
                    flywheel.setTargetRPM(0, 0);
                    follower.followPath(Path8, true);
                    setPathState(8); // Resume normal path sequence
                }
                break;
            case 8:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.setMaxPower(intakespeed);
                    follower.followPath(Path9,true);
                    setPathState(9);
                }
                break;
            case 9:
                if(follower.isBusy()){
                    tubes.setPower(1); // Turn on transfer
                    tubes1.setPower(-1);
                    elev.setPower(-1);
                    intake.setPower(1);
                }

                if(!follower.isBusy()) {
                    /* Grab Sample */
                    tubes.setPower(0); // Turn on transfer
                    tubes1.setPower(0);
                    elev.setPower(0);
                    intake.setPower(0);
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.setMaxPower(normalspeed);
                    follower.followPath(Path10,true);
                    setPathState(10);
                }
                break;
            case 10:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if (follower.getCurrentTValue() > 0.75) {
                    flywheel.setTargetRPM(1225, 0.54); // Start flywheel at 95% of path
                }

                if(!follower.isBusy()) {
                    setPathState(400); // Transition to a custom "Shooting" state
                }
                break;
            case 400:
                follower.holdPoint(pose10); // Keep robot still

                if (flywheel.isReadyToShoot()) {
                    tubes.setPower(1); // Turn on transfer
                    tubes1.setPower(-1);
                    elev.setPower(1);
                    intake.setPower(1);
                }

                if (pathTimer.getElapsedTimeSeconds() > 3) {
                    tubes.setPower(0);
                    tubes1.setPower(0);
                    elev.setPower(0);
                    intake.setPower(0);
                    flywheel.setTargetRPM(0, 0);
                    follower.followPath(Path11, true);
                    setPathState(-1); // Resume normal path sequence
                }
                break;
        }
    }

    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();

        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        flywheel.update();
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();

        draw();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        Drawing.init();

        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Initialize Flywheel and Intake
        flywheel = new FlywheelSubsystem(hardwareMap);
        tubes = hardwareMap.get(DcMotor.class, "tubes");
        tubes1 = hardwareMap.get(CRServo.class, "tubes1");
        elev = hardwareMap.get(DcMotor.class, "elev");
        intake = hardwareMap.get(DcMotor.class, "intake");
        elev.setDirection(DcMotorSimple.Direction.REVERSE);
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

        follower = Constants.createFollower(hardwareMap);
        PanelsConfigurables.INSTANCE.refreshClass(this);
        buildPaths();
        follower.setStartingPose(startPose);
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {
        follower.update();
        drawOnlyCurrent();
    }

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
        follower.setStartingPose(startPose);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {}

    public void drawOnlyCurrent() {
        try {
            Drawing1.drawRobot(follower.getPose());
            Drawing1.sendPacket();
        } catch (Exception e) {
            throw new RuntimeException("Drawing failed " + e);
        }
    }

    public void draw() {
        Drawing1.drawDebug(follower);
    }

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

}
