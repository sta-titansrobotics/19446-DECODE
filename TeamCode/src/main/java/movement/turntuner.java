package movement;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
//import movement.tuning;

@TeleOp
public class turntuner extends LinearOpMode {

    int button2X = 0;
    int button2A = 0;
    int button2B = 0;
    int button2Y = 0;
    int buttonA = 0;
    int buttonB = 0;
    int buttonX = 4;
    int buttonY = 0;

    boolean butAcheck = false;
    boolean butYcheck = false;
    boolean butXcheck = false;
    boolean butBcheck = false;
    boolean but2Acheck = false;
    boolean but2Ycheck = false;
    boolean but2Xcheck = false;
    boolean but2Bcheck = false;

    double prevtime;

    static double dir;
    static double mag;
    static double pi = Math.PI;

    int lexttarg;
    int rexttarg;
    int lexttargfine;
    int rexttargfine;
    double rexterr;
    double lexterr;
    double Lextpower;
    double Rextpower;
    double rextpreverr;
    double lextpreverr;

    double rottarg;
    double totroterr;
    double rotpower;
    double rotpreverr;

    // SHOOTER PID VARIABLES
    double shooterTarg;
    double shooterErr;
    double shooterPower;
    double shooterPrevError;
    double shooterDeltaErr;

    double shooterKp;
    double shooterKd;

    double skd = 0.0002; // increase if not getting steady results????
    double skp = 0.08; // increase if you overshoot


    double torquetarg;

    double rot;

    double offset = 0;
    double oroffset = 0;
    double imureset = 0;
    double sqrt2 = Math.sqrt(2);

    double wristYaw;

    int ypod;
    int xpod;
    double ycord;
    double xcord;

    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    Orientation lastAngles1 = new Orientation();
    double angle;
    double angle1;
    boolean autorot = false;
    boolean targrot = false;
    boolean aprilrot = false;
    double contangle;

    double velo;
    double prevpos;
    double prevtime1;

    double rotvelo;
    double prevpos1;

    double shootp;
    boolean holdrot;
    double apriltuner = 0.7;

    List<AprilTagDetection> detections;

    private Limelight3A limelight3A;
    private LLResult llResult;
    private double distance;


    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;

    @Override
    public void runOpMode() {

        // ----------------------Set Up------------------------------------------------
        // Moving
        DcMotor FL = hardwareMap.get(DcMotor.class, "FL"); // Expansion hub
        DcMotor BL = hardwareMap.get(DcMotor.class, "BL"); // Expansion hub
        DcMotor FR = hardwareMap.get(DcMotor.class, "FR"); // Expantion hub
        DcMotor BR = hardwareMap.get(DcMotor.class, "BR"); // Expantion hub

        DcMotor intake = hardwareMap.get(DcMotor.class, "intake");
        DcMotor elev = hardwareMap.get(DcMotor.class, "elev");
        DcMotor shoot = hardwareMap.get(DcMotor.class, "shoot");

        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");

        LED rled = hardwareMap.get(LED.class, "gled");
        LED gled = hardwareMap.get(LED.class, "rled");
        LED rled1 = hardwareMap.get(LED.class, "gled1");
        LED gled1 = hardwareMap.get(LED.class, "rled1");

        CRServo tubes1 = hardwareMap.get(CRServo.class, "tubes1");
        DcMotor tubes = hardwareMap.get(DcMotor.class, "tubes");
        Servo angles = hardwareMap.get(Servo.class, "angles");

        ColorSensor sensorColor = hardwareMap.get(ColorSensor.class, "sensor_color");
        DistanceSensor sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_color");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        imu = hardwareMap.get(BNO055IMU.class,"imu");

        imu.initialize(parameters);

        /*
         * prevtime = getRuntime();
         * if (getRuntime() - prevtime > 5000)
         */

        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        shoot.setDirection(DcMotorSimple.Direction.REVERSE);
        elev.setDirection(DcMotorSimple.Direction.REVERSE);

        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elev.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        gled.enable(false);
        rled.enable(false);
        gled1.enable(false);
        rled1.enable(false);

        //initAprilTag();

        waitForStart();

        if (isStopRequested())
            return;

        while (opModeIsActive()) {

            offset = (fieldangle() - imureset);
            oroffset = (getAngle()%360) - imureset;

            //imu increases when turning left and decreases when turning right

            //the offset variable tells it how much it has deviated from the original orientation so that it can still move in the correct direction when rotated

            //the dir variable is the variable that determines where we want to be on the sine wave


            if (getRuntime() - prevtime > 10){
                rottarg = 180;
                prevtime = getRuntime();
            }

            if (getRuntime() - prevtime > 5){
                rottarg = 0;
            }

            totroterr = rottarg - (getAngle() % 360);

            // actual pd calculations
            if (totroterr>tuning.rotthresh || totroterr < -tuning.rotthresh) {
                telemetry.addLine("firstpid");
                rotpower = -(totroterr * tuning.rotkp) - ((totroterr - rotpreverr) * tuning.rotkd);

                if (rotpower > 0 && rotpower < tuning.rotmin) {
                    rotpower = tuning.rotmin; // Boost small positive power to minimum positive power
                } else if (rotpower < 0 && rotpower > -tuning.rotmin) {
                    rotpower = -tuning.rotmin; // Boost small negative power to minimum negative power
                }

            } else if (totroterr>tuning.rotthresh2 || totroterr < -tuning.rotthresh2) {
                rotpower = -(totroterr * tuning.rotkp2) - ((totroterr - rotpreverr) * tuning.rotkd2);
                telemetry.addLine("secondpid");
                if (rotpower > 0 && rotpower < tuning.rotmin2) {
                    rotpower = tuning.rotmin2; // Boost small positive power to minimum positive power
                } else if (rotpower < 0 && rotpower > -tuning.rotmin2) {
                    rotpower = -tuning.rotmin2; // Boost small negative power to minimum negative power
                }
            } else {
                rotpower = -(totroterr * tuning.rotkp3) - ((totroterr - rotpreverr) * tuning.rotkd3);
                telemetry.addLine("3st");
                if (rotpower > 0 && rotpower < tuning.rotmin3) {
                    rotpower = tuning.rotmin3; // Boost small positive power to minimum positive power
                } else if (rotpower < 0 && rotpower > -tuning.rotmin3) {
                    rotpower = -tuning.rotmin3; // Boost small negative power to minimum negative power
                }
            }
            // getting the previous error
            rotpreverr = totroterr;

            //will cause the offset to be set back to 0
            if (gamepad1.left_trigger>0.8&&gamepad1.right_trigger>0.8)
                imureset = fieldangle();


            // ------------------DRIVE TRAIN---------------------------------

            telemetry.addLine("olddddddddddddddddddddddddddddddddddddddddddddd");
            dir = Math.atan2(-gamepad1.right_stick_y, gamepad1.right_stick_x) - offset;
            mag = Math.sqrt(Math.pow(gamepad1.right_stick_x, 2) + Math.pow(gamepad1.right_stick_y, 2));
            mag *= sqrt2;
            if (mag > sqrt2)
                mag = sqrt2;

            rot = clamp(rotpower, -1, 1);

            //CHANGE ROTATION TO BE CONTROLLED BY GAMEPAD 2
            if (gamepad1.right_trigger>0.1)
                rot *= 0.5;
            if (gamepad1.right_trigger>0.1)
                mag *= 0.5;


            if (gamepad1.right_stick_y != 0 || gamepad1.right_stick_x != 0 || gamepad1.left_stick_x != 0 || rot != 0) {
                FR.setPower((Math.sin(dir - (pi / 4)) * mag) - rot);
                FL.setPower((Math.sin(dir + (pi / 4)) * mag) + rot);
                BR.setPower((Math.sin(dir + (pi / 4)) * mag) - rot);
                BL.setPower((Math.sin(dir - (pi / 4)) * mag) + rot);
            } else {
                FL.setPower(0);
                BL.setPower(0);
                FR.setPower(0);
                BR.setPower(0);
            }

            telemetry.addLine("Drivetrain");
            telemetry.addLine("");
            telemetry.addData("FR Power", FR.getPower());
            telemetry.addData("FL Power", FL.getPower());
            telemetry.addData("BR Power", BR.getPower());
            telemetry.addData("BL Power", BL.getPower());

            telemetry.addLine("");
            telemetry.addLine("");
            telemetry.addLine("");

            telemetry.addData("rottarg", rottarg);
            telemetry.addData("timer", getRuntime());
            telemetry.addData("prevtime", prevtime);
            telemetry.addData("totroterr", totroterr);
            telemetry.addData("totroterr", totroterr);
            telemetry.addData("rotpower", rotpower);
            telemetry.addData("rotpreverr", rotpreverr);
            telemetry.addData("rot", rot);
            telemetry.addData("getangle", getAngle());
            telemetry.addData("compensat", Math.pow((1.0175),rotvelo));
            telemetry.addData("fieldangle", fieldangle());
            telemetry.addData("offset", offset);
            telemetry.addData("oroffset", oroffset);
            telemetry.addData("contangle", contangle);
            telemetry.addData("dir", dir);



            telemetry.addLine("");
            telemetry.addLine("<------------------->");
            telemetry.addLine("");

            telemetry.update();
        }
    }

    private double getAngle() {
        //this converts the imu's outputs from -180 to 180 into an output of 0 to 360

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        angle += deltaAngle;

        lastAngles = angles;


        return angle;
    }

    private double fieldangle() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

        double deltaAngle = angles.firstAngle - lastAngles1.firstAngle;

        angle1 += deltaAngle;

        lastAngles1 = angles;

        return angle1;
    }
    public double clamp(double value, double min, double max) {
        return Math.min(Math.max(value, min), max);
    }

    private void updateLimelight() {
        limelight3A.updateRobotOrientation(getAngle());

        llResult = limelight3A.getLatestResult();

        if (llResult != null && llResult.isValid()) {
            // Iterate through all detected fiducials (AprilTags)
            for (com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult fiducial : llResult.getFiducialResults()) {
                int id = fiducial.getFiducialId();

                // Only update distance if it's Tag 20 or Tag 24
                if (id == 20 || id == 24) {
                    // The Limelight SDK gives us target area as a normalized value (0-1),
                    // but our formula needs it as a percentage (0-100).
                    distance = getDistanceFromTag(fiducial.getTargetArea() * 100.0);
                    telemetry.addData("Target Area", fiducial.getTargetArea());


                    return; // Exit once we found our target tag
                }
            }
        }
        // If no valid tag is found, reset distance to 0 to avoid using stale values.
        distance = 0;
    }

    public double getDistanceFromTag(double ta){
        // This equation is derived from a power regression of distance vs. target area
        // from a table of observed values.
        return 157.6 * Math.pow(ta, -0.584);
    }
}