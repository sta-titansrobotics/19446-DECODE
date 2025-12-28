package org.firstinspires.ftc.teamcode.pedroPathing; // make sure this aligns with class location

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.PoseHistory;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

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
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "from-----------------------------------scratch", group = "pedropathing")
public class scratch extends OpMode {
    public Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    static TelemetryManager telemetryM;

    // --- Pose Definitions (Organized at the beginning) ---
    private final Pose startPose = new Pose(110.000, 135.000, Math.toRadians(90));
    private final Pose pose1     = new Pose(100.6, 99.1,   Math.toRadians(50));
    private final Pose pose2     = new Pose(97.9, 83.5,    Math.toRadians(0));
    private final Pose pose3     = new Pose(125, 83.3,     Math.toRadians(0));
    private final Pose pose4     = new Pose(91.7, 92,      Math.toRadians(42));
    private final Pose pose5     = new Pose(98.6, 59.3,    Math.toRadians(0));
    private final Pose pose6     = new Pose(125, 59.5,     Math.toRadians(0));
    private final Pose pose7     = new Pose(87.2, 86.3,    Math.toRadians(42));
    private final Pose pose8     = new Pose(101.9, 35.5,   Math.toRadians(0));
    private final Pose pose9     = new Pose(125, 35.4,     Math.toRadians(0));
    private final Pose pose10    = new Pose(89.2, 89.4,    Math.toRadians(45));
    private final Pose pose11    = new Pose(100, 135,      Math.toRadians(180));

    private Path startpath;
    private PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9, Path10, Path11;

    private FlywheelSubsystem flywheel;
    private DcMotor tubes;
    private DcMotor elev;
    private DcMotor intake;
    private CRServo tubes1;

    // Constants for the shoot
    private final double SHOOT_RPM = 1250;
    private final double SHOOT_ANGLE_POS = 0.5;
    private final int NUM_BALLS = 3;

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
                follower.setMaxPower(0.7);
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
                    flywheel.setTargetRPM(1150, 0.65); // Start flywheel at 95% of path
                }

                if(!follower.isBusy()) {
                    setPathState(100); // Transition to a custom "Shooting" state
                }

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */

                break;
            case 100:
                follower.holdPoint(pose1); // Keep robot still

                if (flywheel.isReadyToShoot()) {
                    tubes.setPower(1.0); // Turn on transfer
                    tubes1.setPower(-1.0);
                    elev.setPower(1.0);
                }

                if (pathTimer.getElapsedTimeSeconds() > 3.5) {
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
                    follower.setMaxPower(0.4);
                    follower.followPath(Path3,true);
                    setPathState(3);
                }
                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */

                if(follower.isBusy()){
                    tubes.setPower(1.0); // Turn on transfer
                    tubes1.setPower(-1.0);
                    elev.setPower(-1.0);
                    intake.setPower(1);
                }

                if(!follower.isBusy()) {
                    /* Score Sample */
                    tubes.setPower(0); // Turn on transfer
                    tubes1.setPower(0);
                    elev.setPower(0);
                    intake.setPower(0);

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.setMaxPower(0.7);
                    follower.followPath(Path4,true);
                    setPathState(4);
                }
                break;
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */

                if (follower.getCurrentTValue() > 0.75) {
                    flywheel.setTargetRPM(1150, 0.65); // Start flywheel at 95% of path
                }

                if(!follower.isBusy()) {
                    setPathState(200); // Transition to a custom "Shooting" state
                }
                break;
            case 200:
                follower.holdPoint(pose4); // Keep robot still

                if (flywheel.isReadyToShoot()) {
                    tubes.setPower(1.0); // Turn on transfer
                    tubes1.setPower(-1.0);
                    elev.setPower(1.0);
                }

                if (pathTimer.getElapsedTimeSeconds() > 3.5) {
                    tubes.setPower(0);
                    tubes1.setPower(0);
                    elev.setPower(0);
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
                    follower.setMaxPower(0.4);
                    follower.followPath(Path6,true);
                    setPathState(6);
                }
                break;
            case 6:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(follower.isBusy()){
                    tubes.setPower(1.0); // Turn on transfer
                    tubes1.setPower(-1.0);
                    elev.setPower(-1.0);
                    intake.setPower(1);
                }

                if(!follower.isBusy()) {
                    /* Grab Sample */
                    tubes.setPower(0); // Turn on transfer
                    tubes1.setPower(0);
                    elev.setPower(0);
                    intake.setPower(0);
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.setMaxPower(0.7);
                    follower.followPath(Path7,true);
                    setPathState(7);
                }
                break;
            case 7:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if (follower.getCurrentTValue() > 0.75) {
                    flywheel.setTargetRPM(1150, 0.65); // Start flywheel at 95% of path
                }

                if(!follower.isBusy()) {
                    setPathState(300); // Transition to a custom "Shooting" state
                }
                break;
            case 300:
                follower.holdPoint(pose7); // Keep robot still

                if (flywheel.isReadyToShoot()) {
                    tubes.setPower(1.0); // Turn on transfer
                    tubes1.setPower(-1.0);
                    elev.setPower(1.0);
                }

                if (pathTimer.getElapsedTimeSeconds() > 3.5) {
                    tubes.setPower(0);
                    tubes1.setPower(0);
                    elev.setPower(0);
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
                    follower.setMaxPower(0.4);
                    follower.followPath(Path9,true);
                    setPathState(9);
                }
                break;
            case 9:
                if(follower.isBusy()){
                    tubes.setPower(1.0); // Turn on transfer
                    tubes1.setPower(-1.0);
                    elev.setPower(-1.0);
                    intake.setPower(1);
                }

                if(!follower.isBusy()) {
                    /* Grab Sample */
                    tubes.setPower(0); // Turn on transfer
                    tubes1.setPower(0);
                    elev.setPower(0);
                    intake.setPower(0);
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.setMaxPower(0.7);
                    follower.followPath(Path10,true);
                    setPathState(10);
                }
                break;
            case 10:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if (follower.getCurrentTValue() > 0.75) {
                    flywheel.setTargetRPM(1150, 0.65); // Start flywheel at 95% of path
                }

                if(!follower.isBusy()) {
                    setPathState(400); // Transition to a custom "Shooting" state
                }
                break;
            case 400:
                follower.holdPoint(pose10); // Keep robot still

                if (flywheel.isReadyToShoot()) {
                    tubes.setPower(1.0); // Turn on transfer
                    tubes1.setPower(-1.0);
                    elev.setPower(1.0);
                }

                if (pathTimer.getElapsedTimeSeconds() > 3.5) {
                    tubes.setPower(0);
                    tubes1.setPower(0);
                    elev.setPower(0);
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

class Drawing1 {
    public static final double ROBOT_RADIUS = 9; // woah
    private static final FieldManager panelsField = PanelsField.INSTANCE.getField();

    private static final Style robotLook = new Style(
            "", "#3F51B5", 1
    );
    private static final Style historyLook = new Style(
            "", "#4CAF50", 1
    );

    /**
     * This prepares Panels Field for using Pedro Offsets
     */
    public static void init() {
        panelsField.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
    }

    /**
     * This draws everything that will be used in the Follower's telemetryDebug() method. This takes
     * a Follower as an input, so an instance of the DashboardDrawingHandler class is not needed.
     *
     * @param follower Pedro Follower instance.
     */
    public static void drawDebug(Follower follower) {
        if (follower.getCurrentPath() != null) {
            drawPath(follower.getCurrentPath(), robotLook);
            Pose closestPoint = follower.getPointFromPath(follower.getCurrentPath().getClosestPointTValue());
            drawRobot(new Pose(closestPoint.getX(), closestPoint.getY(), follower.getCurrentPath().getHeadingGoal(follower.getCurrentPath().getClosestPointTValue())), robotLook);
        }
        drawPoseHistory(follower.getPoseHistory(), historyLook);
        drawRobot(follower.getPose(), historyLook);

        sendPacket();
    }

    /**
     * This draws a robot at a specified Pose with a specified
     * look. The heading is represented as a line.
     *
     * @param pose  the Pose to draw the robot at
     * @param style the parameters used to draw the robot with
     */
    public static void drawRobot(Pose pose, Style style) {
        if (pose == null || Double.isNaN(pose.getX()) || Double.isNaN(pose.getY()) || Double.isNaN(pose.getHeading())) {
            return;
        }

        panelsField.setStyle(style);
        panelsField.moveCursor(pose.getX(), pose.getY());
        panelsField.circle(ROBOT_RADIUS);

        Vector v = pose.getHeadingAsUnitVector();
        v.setMagnitude(v.getMagnitude() * ROBOT_RADIUS);
        double x1 = pose.getX() + v.getXComponent() / 2, y1 = pose.getY() + v.getYComponent() / 2;
        double x2 = pose.getX() + v.getXComponent(), y2 = pose.getY() + v.getYComponent();

        panelsField.setStyle(style);
        panelsField.moveCursor(x1, y1);
        panelsField.line(x2, y2);
    }

    /**
     * This draws a robot at a specified Pose. The heading is represented as a line.
     *
     * @param pose the Pose to draw the robot at
     */
    public static void drawRobot(Pose pose) {
        drawRobot(pose, robotLook);
    }

    /**
     * This draws a Path with a specified look.
     *
     * @param path  the Path to draw
     * @param style the parameters used to draw the Path with
     */
    public static void drawPath(Path path, Style style) {
        double[][] points = path.getPanelsDrawingPoints();

        for (int i = 0; i < points[0].length; i++) {
            for (int j = 0; j < points.length; j++) {
                if (Double.isNaN(points[j][i])) {
                    points[j][i] = 0;
                }
            }
        }

        panelsField.setStyle(style);
        panelsField.moveCursor(points[0][0], points[0][1]);
        panelsField.line(points[1][0], points[1][1]);
    }

    /**
     * This draws all the Paths in a PathChain with a
     * specified look.
     *
     * @param pathChain the PathChain to draw
     * @param style     the parameters used to draw the PathChain with
     */
    public static void drawPath(PathChain pathChain, Style style) {
        for (int i = 0; i < pathChain.size(); i++) {
            drawPath(pathChain.getPath(i), style);
        }
    }

    /**
     * This draws the pose history of the robot.
     *
     * @param poseTracker the PoseHistory to get the pose history from
     * @param style       the parameters used to draw the pose history with
     */
    public static void drawPoseHistory(PoseHistory poseTracker, Style style) {
        panelsField.setStyle(style);

        int size = poseTracker.getXPositionsArray().length;
        for (int i = 0; i < size - 1; i++) {

            panelsField.moveCursor(poseTracker.getXPositionsArray()[i], poseTracker.getYPositionsArray()[i]);
            panelsField.line(poseTracker.getXPositionsArray()[i + 1], poseTracker.getYPositionsArray()[i + 1]);
        }
    }

    /**
     * This draws the pose history of the robot.
     *
     * @param poseTracker the PoseHistory to get the pose history from
     */
    public static void drawPoseHistory(PoseHistory poseTracker) {
        drawPoseHistory(poseTracker, historyLook);
    }

    /**
     * This tries to send the current packet to FTControl Panels.
     */
    public static void sendPacket() {
        panelsField.update();
    }


}


