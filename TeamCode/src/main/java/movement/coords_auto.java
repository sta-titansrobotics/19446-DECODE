package movement;

import android.util.Size;

import androidx.core.graphics.ColorUtils;

import com.acmerobotics.dashboard.config.Config;
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
import java.util.List;
import com.acmerobotics.dashboard.FtcDashboard;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import com.acmerobotics.dashboard.FtcDashboard;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import com.qualcomm.robotcore.hardware.ColorSensor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import com.qualcomm.robotcore.hardware.LED;
//import movement.tuning;

@TeleOp
public class coords_auto extends LinearOpMode {

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
    double roterr;
    double rotpower;
    double rotpreverr;
    double totroterr;
    double rotinteg;

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
    int id;
    boolean shootenabled = false;
    boolean pidActiveLastLoop = false;
    double lastTime = 0;


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
        limelight3A.pipelineSwitch(0);
        limelight3A.start();

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
        FtcDashboard dashboard = FtcDashboard.getInstance();

        waitForStart();

        if (isStopRequested())
            return;

        while (opModeIsActive()) {

            double currentTime = getRuntime();
            double dt = currentTime - lastTime;
            lastTime = currentTime;

            updateLimelight();

            offset = (fieldangle() - imureset);
            oroffset = (getAngle()%360) - imureset;

            //imu increases when turning left and decreases when turning right

            //the offset variable tells it how much it has deviated from the original orientation so that it can still move in the correct direction when rotated

            //the dir variable is the variable that determines where we want to be on the sine wave

            totroterr = rottarg - (getAngle() % 360);

            //will cause the offset to be set back to 0
            if (gamepad1.left_trigger>0.8&&gamepad1.right_trigger>0.8)
                imureset = fieldangle();

            if (gamepad2.right_trigger>0.3){
                if (velo > 1100){
                    //if (sensorColor.alpha()<75||sensorDistance.getDistance(DistanceUnit.CM)>4.5){
                        elev.setPower(1);
                    //}
                } else {
                    elev.setPower(-0.7);
                }
            } else {
                elev.setPower(0);
            }

            tubes.setPower(gamepad2.right_trigger*0.6);

            tubes1.setPower(-gamepad2.right_trigger);

//            if (gamepad2.dpad_up) {
//                shooterTarg = 1300;
//                angles.setPosition(0.6);
//            } else if (gamepad2.dpad_right) {
//                shooterTarg = 1150;
//                angles.setPosition(0.4);
//            } else if (gamepad2.dpad_left) {
//                angles.setPosition(0.85);
//                shooterTarg = 1550;
//            } else if (gamepad2.dpad_down) {
//                shooterTarg = 0;
//                angles.setPosition(0);
//            }


            if (gamepad2.dpad_up) {
                shootenabled = true;
            } else if (gamepad2.dpad_down) {
                shootenabled = false;
                shooterTarg = 0;
                angles.setPosition(0);
            }

            if (shootenabled) {
                shooterTarg = (tuning.shootconst+(2.71*distance)+((-0.00217)*(Math.pow(distance,2))));
                angles.setPosition(tuning.angletune);
                /*
                if (velo < shooterTarg)
                    angles.setPosition((26.5 - (0.0404 * velo) + (0.0000154 * Math.pow(velo, 2))) + (2.75 + (0.067 * (velo-shooterTarg)) + (0.0004 * Math.pow((velo-shooterTarg), 2))));
                else
                    angles.setPosition(26.5 - (0.0404 * velo) + (0.0000154 * Math.pow(velo, 2)));
                 */
            }



            // SHOOTER PID

            shooterErr = shooterTarg - velo; // P: current position - desired position aka current error

            shooterPower = (skp * (shooterErr)) + (skd * shooterPrevError); // multiplies the tuning values by teh error

            if (gamepad2.right_trigger<0.3) {
                if (shooterTarg < 1150 && shooterTarg != 0){
                    shoot.setPower(clamp((shooterPower), 0.3, 0.82));
                } else if (shooterTarg >= 1150 && shooterTarg < 1350){
                    shoot.setPower(clamp((shooterPower), 0.4, 0.82));
                } else if (shooterTarg >= 1350 && shooterTarg < 1700)
                    shoot.setPower(clamp((shooterPower), 0.55, 0.77));
                else if (shooterTarg > 1700)
                    shoot.setPower(clamp((shooterPower), 0.77, 1));
                else if (shooterTarg == 0)
                    shoot.setPower(0);
            } else {
                if (shooterTarg < 1150 && shooterTarg != 0){
                    shoot.setPower(clamp((shooterPower), 0.3, 0.82));
                } else if (shooterTarg >= 1150 && shooterTarg < 1350){
                    shoot.setPower(clamp((shooterPower), 0.4, 0.82));
                } else if (shooterTarg >= 1350 && shooterTarg < 1700)
                    shoot.setPower(clamp((shooterPower), 0.575, 0.82));
                else if (shooterTarg > 1700)
                    shoot.setPower(clamp((shooterPower), 0.82, 1));
                else if (shooterTarg == 0)
                    shoot.setPower(0);
            }



            shooterPrevError = shooterErr;

            telemetry.addLine("shooting pid");
            telemetry.addData("shoot", shoot.getPower());
            telemetry.addData("shooterr", shooterErr);
            telemetry.addData("shootpower", shooterPower);
            telemetry.addData("shootpreverr", shooterPrevError);
            telemetry.addData("shoottarg", shooterTarg);
            telemetry.addData("shootvelo", velo);
            telemetry.addData("distance", distance);

            telemetry.addLine("");

            telemetry.addLine("Raw Sensor Values");
            telemetry.addData("Red", sensorColor.red());
            telemetry.addData("Green", sensorColor.green());
            telemetry.addData("Blue", sensorColor.blue());
            telemetry.addData("Ambient (Alpha)", sensorColor.alpha());
            telemetry.addData("Distance", sensorDistance.getDistance(DistanceUnit.CM));

            telemetry.addLine("");

            telemetry.addLine("shoot");
            telemetry.addData("shoot", shoot.getPower());
            telemetry.addData("elev", elev.getPower());
            telemetry.addData("tubes", tubes.getPower());
            telemetry.addData("angles", angles.getPosition());
            dashboard.getTelemetry().addData("rot", rot);
            dashboard.getTelemetry().addData("tx", -llResult.getTx());
            dashboard.getTelemetry().addData("integ",rotinteg);

            telemetry.addLine("");

            velo = (shoot.getCurrentPosition()-prevpos)/(getRuntime() - prevtime1);
            rotvelo = (getAngle()-prevpos1)/(getRuntime() - prevtime1);

            prevtime1 = getRuntime();
            prevpos = shoot.getCurrentPosition();
            prevpos1 = getAngle();
            telemetry.addData("velo", velo);
            telemetry.addData("pos", shoot.getCurrentPosition());
            telemetry.addData("rotvelo", rotvelo);
            telemetry.addData("pos1", getAngle());

            telemetry.addLine("");


            if (300 > shooterErr && shooterErr > -300){
                gled.enable(false);
                rled.enable(true);
                gled1.enable(false);
                rled1.enable(true);
            } else {
                gled.enable(true);
                rled.enable(false);
                gled1.enable(true);
                rled1.enable(false);
            }

            intake.setPower(gamepad2.right_trigger-gamepad2.left_trigger);


            telemetry.addLine("aprilrot");
            if (dt <= 0) dt = 0.001;

            if (gamepad1.dpad_up && llResult != null && llResult.isValid()){
                boolean targetDetected = false;

                // Keep your original TX finding logic
                if (id == 20 || id == 24) {
                    roterr = -llResult.getTx();
                    rot = (clamp(rotpower, -1, 1));
                    telemetry.addLine("~~~~~~~~~~~~~~~~~~detecting~~~~~~~~~~~~~~~~~~~~~");
                    targetDetected = true;
                }

                if (targetDetected) {
                    // 1. First-time Initialization to prevent "Derivative Kick"
                    if (!pidActiveLastLoop) {
                        rotpreverr = roterr;
                        rotinteg = 0;
                        pidActiveLastLoop = true;
                    }

                    // 2. Deadband: Stop if within +/- 2 degrees
                    if (Math.abs(roterr) < 2.0) {
                        rotpower = 0;
                        rotinteg = 0;
                    } else {
                        // 2. Integral Logic: Only accumulate when close (prevents windup)
                        if (Math.abs(roterr) < 5.0) {
                            rotinteg += (roterr * dt); // Multiplied by dt for time-consistency
                        } else {
                            rotinteg = 0;
                        }

                        // 3. Derivative Calculation: Change in error over time
                        double derivative = (roterr - rotpreverr) / dt;

                        // 4. PID Calculation (Using your existing stage logic)
                        if (Math.abs(roterr) < tuning.rotthresh2 && tuning.stage >= 2) {
                            rotpower = -(roterr * tuning.rotkp2) - (derivative * tuning.rotkd2) + (rotinteg * tuning.rotki2);
                            telemetry.addLine("222222222222222222222222222222222222");
                            if (Math.abs(rotpower) < tuning.rotmin2) rotpower = Math.signum(rotpower) * tuning.rotmin2;
                        }
                        else if (Math.abs(roterr) < tuning.rotthresh && tuning.stage == 3) {
                            rotpower = -(roterr * tuning.rotkp) - (derivative * tuning.rotkd) + (rotinteg * tuning.rotki);
                            telemetry.addLine("111111111111111111111111111111111111111111111");
                            if (Math.abs(rotpower) < tuning.rotmin) rotpower = Math.signum(rotpower) * tuning.rotmin;
                        }
                        else if (tuning.stage >= 1) {
                            rotpower = -(roterr * tuning.rotkp3) - (derivative * tuning.rotkd3) + (rotinteg * tuning.rotki3);
                            telemetry.addLine("33333333333333333333333333333333333333333333333333");
                            if (Math.abs(rotpower) < tuning.rotmin3) rotpower = Math.signum(rotpower) * tuning.rotmin3;
                        }
                    }

                    rot = clamp(rotpower, -1, 1);
                    rotpreverr = roterr; // Update for the next loop's D-term

                } else {
                    // Button pressed but target NOT found: Force Stop
                    rot = 0;
                    rotinteg = 0;
                    pidActiveLastLoop = false;
                    telemetry.addLine("!!! TARGET LOST - STOPPING !!!");
                }
            } else {
                // Manual Control
                rot = gamepad1.left_stick_x;
                rotinteg = 0;
                pidActiveLastLoop = false;
            }
            /*
            if (gamepad1.dpad_up && llResult != null && llResult.isValid()){
                if (id == 20 || id == 24) {
                    roterr = -llResult.getTx();
                    rot = (clamp(rotpower, -1, 1));
                    telemetry.addLine("~~~~~~~~~~~~~~~~~~detecting~~~~~~~~~~~~~~~~~~~~~");
                }
                // actual pd calculations
                if (Math.abs(roterr)<tuning.rotthresh2 && tuning.stage >= 2) {
                    rotpower = -(roterr * tuning.rotkp2) - ((roterr - rotpreverr) * tuning.rotkd2) + (rotinteg * tuning.rotki2);
                    if (Math.abs(roterr) > 0.25)
                        rotinteg += roterr;
                    else
                        rotinteg = 0;
                    telemetry.addLine("secondpid");
                    if (rotpower > 0 && rotpower < tuning.rotmin2) {
                        rotpower = tuning.rotmin2; // Boost small positive power to minimum positive power
                    } else if (rotpower < 0 && rotpower > -tuning.rotmin2) {
                        rotpower = -tuning.rotmin2; // Boost small negative power to minimum negative power
                    }
                } else if (Math.abs(roterr)<tuning.rotthresh && tuning.stage == 3) {
                    telemetry.addLine("firstpid");
                    if (Math.abs(roterr)>0.25)
                        rotinteg += roterr;
                    else
                        rotinteg = 0;
                    rotpower = -(roterr * tuning.rotkp) - ((roterr - rotpreverr) * tuning.rotkd) + (rotinteg*tuning.rotki);

                    if (rotpower > 0 && rotpower < tuning.rotmin) {
                        rotpower = tuning.rotmin; // Boost small positive power to minimum positive power
                    } else if (rotpower < 0 && rotpower > -tuning.rotmin) {
                        rotpower = -tuning.rotmin; // Boost small negative power to minimum negative power
                    }

                } else if (tuning.stage >= 1) {
                    rotpower = -(roterr * tuning.rotkp3) - ((roterr - rotpreverr) * tuning.rotkd3) + (rotinteg*tuning.rotki3);
                    if (Math.abs(roterr)>0.25)
                        rotinteg += roterr;
                    else
                        rotinteg = 0;
                    telemetry.addLine("3st");
                    if (rotpower > 0 && rotpower < tuning.rotmin3) {
                        rotpower = tuning.rotmin3; // Boost small positive power to minimum positive power
                    } else if (rotpower < 0 && rotpower > -tuning.rotmin3) {
                        rotpower = -tuning.rotmin3; // Boost small negative power to minimum negative power
                    }
                }
                // getting the previous error
                rotpreverr = roterr;


            } else {
                rot = gamepad1.left_stick_x;
                rotinteg = 0;
            }

             */


            // ------------------DRIVE TRAIN---------------------------------

            telemetry.addLine("olddddddddddddddddddddddddddddddddddddddddddddd");
            dir = Math.atan2(-gamepad1.right_stick_y, gamepad1.right_stick_x) - offset;
            mag = Math.sqrt(Math.pow(gamepad1.right_stick_x, 2) + Math.pow(gamepad1.right_stick_y, 2));
            mag *= sqrt2;
            if (mag > sqrt2)
                mag = sqrt2;


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
            telemetry.addData("tx", llResult.getTx());
            telemetry.addData("totroterr", totroterr);
            telemetry.addData("roterr", roterr);
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

            //telemetry.addData("", );

            //-----------------apriltagsssssssss------------------------------------

            telemetry.update();
            dashboard.getTelemetry().update();
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
                id = fiducial.getFiducialId();

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