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
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;

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

        LED rled = hardwareMap.get(LED.class, "gled");
        LED gled = hardwareMap.get(LED.class, "rled");
        LED rled1 = hardwareMap.get(LED.class, "gled1");
        LED gled1 = hardwareMap.get(LED.class, "rled1");

        CRServo tubes1 = hardwareMap.get(CRServo.class, "tubes1");
        DcMotor tubes = hardwareMap.get(DcMotor.class, "tubes");
        Servo angles = hardwareMap.get(Servo.class, "shoot");

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

        initAprilTag();

        waitForStart();

        if (isStopRequested())
            return;

        while (opModeIsActive()) {

            offset = (fieldangle() - imureset);
            oroffset = (getAngle()%360) - imureset;

            //imu increases when turning left and decreases when turning right

            //the offset variable tells it how much it has deviated from the original orientation so that it can still move in the correct direction when rotated

            //the dir variable is the variable that determines where we want to be on the sine wave

            totroterr = rottarg - (getAngle() % 360);

            if (!aprilrot)
                roterr = totroterr - 360.0 * Math.floor((totroterr + 180.0) / 360.0);

            // actual pd calculations
            if (roterr>tuning.rotthresh || roterr < -tuning.rotthresh) {
                telemetry.addLine("firstpid");
                rotpower = -(roterr * tuning.rotkp) - ((roterr - rotpreverr) * tuning.rotkd);

                if (rotpower > 0 && rotpower < tuning.rotmin) {
                    rotpower = tuning.rotmin; // Boost small positive power to minimum positive power
                } else if (rotpower < 0 && rotpower > -tuning.rotmin) {
                    rotpower = -tuning.rotmin; // Boost small negative power to minimum negative power
                }

            } else if (roterr>tuning.rotthresh2 || roterr < -tuning.rotthresh2) {
                rotpower = -(roterr * tuning.rotkp2) - ((roterr - rotpreverr) * tuning.rotkd2);
                telemetry.addLine("secondpid");
                if (rotpower > 0 && rotpower < tuning.rotmin2) {
                    rotpower = tuning.rotmin2; // Boost small positive power to minimum positive power
                } else if (rotpower < 0 && rotpower > -tuning.rotmin2) {
                    rotpower = -tuning.rotmin2; // Boost small negative power to minimum negative power
                }
            } else {
                rotpower = -(roterr * tuning.rotkp3) - ((roterr - rotpreverr) * tuning.rotkd3);
                telemetry.addLine("3st");
                if (rotpower > 0 && rotpower < tuning.rotmin3) {
                    rotpower = tuning.rotmin3; // Boost small positive power to minimum positive power
                } else if (rotpower < 0 && rotpower > -tuning.rotmin3) {
                    rotpower = -tuning.rotmin3; // Boost small negative power to minimum negative power
                }
            }
            // getting the previous error
            rotpreverr = roterr;

            if (holdrot){
                if (rotvelo>0)
                    rottarg = getAngle()+Math.pow((1.0175),rotvelo);
                else if (rotvelo<0)
                    rottarg = getAngle()-Math.pow((1.0175),rotvelo);
                holdrot = false;
            }

            //will cause the offset to be set back to 0
            if (gamepad1.left_trigger>0.8&&gamepad1.right_trigger>0.8)
                imureset = fieldangle();

            if (autorot) {

                if (gamepad1.dpad_up)
                    rottarg = getAngle() - oroffset;
                else if (gamepad1.dpad_down)
                    rottarg = getAngle() - oroffset - 180;
                else if (gamepad1.dpad_left)
                    rottarg = getAngle() - oroffset + 45;
                else if (gamepad1.dpad_right)
                    rottarg = getAngle() - oroffset - 135;
            } else if (targrot){
                //contangle += 5*gamepad1.left_stick_x;

                if (gamepad1.left_stick_x !=0){
                    rot = gamepad1.left_stick_x;
                    holdrot = true;
                    //contangle = (getAngle()%360);
                    //rottarg = getAngle();
                    rotpower = 0;
                }

            }

            // 4 stage sliders
            // limiting the motors movement so that it does not try to over extend the slider



            //to fix starting jitter
            /*
            if (getAngle() < 10 && rottarg == 1900){
                rotpower = 0;
            }
            */

            // actually setting the motor power

            //rot = gamepad1.left_stick_x;




            FtcDashboard.getInstance().startCameraStream(visionPortal, 10); // Stream at 30 FPS

            if (gamepad2.right_trigger>0.3){
                if (velo > 1100){
                    if (sensorColor.alpha()<75||sensorDistance.getDistance(DistanceUnit.CM)>4.5){
                        elev.setPower(1);
                    }
                } else {
                    elev.setPower(-0.7);
                }
            } else {
                elev.setPower(0);
            }

            tubes.setPower(gamepad2.right_trigger);

            tubes1.setPower(-gamepad2.right_trigger);

            //shoot.setPower(gamepad1.left_trigger);
            //if (gamepad2.b)
            //    angles.setPosition(gamepad2.left_trigger);

            if (gamepad2.dpad_up) {
                shooterTarg = 1300;
                angles.setPosition(0.5);
            }
            else if (gamepad2.dpad_right) {
                shooterTarg = 1150;
                angles.setPosition(0.75);
            }
            else if (gamepad2.dpad_left) {
                angles.setPosition(0.5);
                shooterTarg = 1550;
            } else if (gamepad2.dpad_down) {
                shooterTarg = 0;
                angles.setPosition(0);
            }

            // SHOOTER PID

            shooterErr = shooterTarg - velo; // P: current position - desired position aka current error

            shooterPower = (skp * (shooterErr)) + (skd * shooterPrevError); // multiplies the tuning values by teh error

            if (shooterTarg>0)
                shoot.setPower(clamp((shooterPower), 0.55, 0.77));
            else if (shooterTarg == 0)
                shoot.setPower(0);



            shooterPrevError = shooterErr;

            telemetry.addLine("shooting pid");
            telemetry.addData("shoot", shoot.getPower());
            telemetry.addData("shooterr", shooterErr);
            telemetry.addData("shootpower", shooterPower);
            telemetry.addData("shootpreverr", shooterPrevError);
            telemetry.addData("shoottarg", shooterTarg);
            telemetry.addData("shootvelo", velo);

            telemetry.addLine("Raw Sensor Values");
            telemetry.addData("Red", sensorColor.red());
            telemetry.addData("Green", sensorColor.green());
            telemetry.addData("Blue", sensorColor.blue());
            telemetry.addData("Ambient (Alpha)", sensorColor.alpha());
            telemetry.addData("Distance", sensorDistance.getDistance(DistanceUnit.CM));



            telemetry.addLine("shoot");
            telemetry.addData("shoot", shoot.getPower());
            telemetry.addData("elev", elev.getPower());
            telemetry.addData("tubes", tubes.getPower());
            telemetry.addData("angles", angles.getPosition());



            velo = (shoot.getCurrentPosition()-prevpos)/(getRuntime() - prevtime1);
            rotvelo = (getAngle()-prevpos1)/(getRuntime() - prevtime1);

            prevtime1 = getRuntime();
            prevpos = shoot.getCurrentPosition();
            prevpos1 = getAngle();
            telemetry.addData("velo", velo);
            telemetry.addData("pos", shoot.getCurrentPosition());
            telemetry.addData("rotvelo", rotvelo);
            telemetry.addData("pos1", getAngle());


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
//a;ldjf

            if (gamepad1.x && !butXcheck) {
                buttonX += 1;
                if (buttonX > 4)
                    buttonX = 1;
                butXcheck = true;
            }

            if (!gamepad1.x) {
                butXcheck = false;
            }

            if (butXcheck) {
                if (buttonX % 4 == 0) {
                    autorot = false;
                    targrot = false;
                    aprilrot = true;
                    telemetry.addLine("aprilrot");
                } else if (buttonX % 3 == 0) {
                    autorot = false;
                    targrot = true;
                    aprilrot = false;
                    telemetry.addLine("targrot");
                } else if (buttonX % 2 == 0) {
                    autorot = true;
                    targrot = false;
                    aprilrot = false;
                    telemetry.addLine("autorot");
                } else {
                    targrot = false;
                    autorot = false;
                    aprilrot = false;
                    telemetry.addLine("mantual rot");
                }
            }

            intake.setPower(gamepad2.right_trigger);

            if (autorot) {
                rot = (clamp(rotpower, -1, 1));
                telemetry.addLine("");
                telemetry.addLine("autorot");
            } else if (targrot) {
                if (gamepad1.left_stick_x != 0)
                    rot = gamepad1.left_stick_x;
                else
                    rot = (clamp(rotpower, -1, 1));
                telemetry.addLine("");
                telemetry.addLine("targrot");
            }else if(aprilrot){
                for (AprilTagDetection tag : detections) {
                    if (tag.id == 22) {
                        double tagX = tag.center.x;
                        double tagY = tag.center.y;
                        double dx = tagX - 320;
                        double dy = tagY - 240;
                        double pixelDistance = dx;
                        roterr = (-pixelDistance * tuning.aprilcorr);
                        rot = (clamp(rotpower, -1, 1));
                        telemetry.addLine("");
                        telemetry.addLine("aprilrot");
                        telemetry.addData("april err", pixelDistance);
                        telemetry.addLine("");
                    } else {
                        rottarg = getAngle() % 360;
                        //targrot = true;
                        //aprilrot= false;
                        rot = (clamp(rotpower, -1, 1));
                    }
                }
            } else {
                rot = gamepad1.left_stick_x;
                telemetry.addLine("");
                telemetry.addLine("manual rot");
            }
//a;lkdfja;
/*
            contangle += 5*gamepad1.left_stick_x;
            if (gamepad2.x && !but2Xcheck) {
                button2X += 1;
                if (button2X > 3)
                    button2X = 1;
                but2Xcheck = true;
            }

            if (!gamepad2.x) {
                but2Xcheck = false;
            }

            if (but2Xcheck) {
                if (button2X % 3 == 0) {
                    autorot = false;
                    targrot = true;
                    telemetry.addLine("targrot");
                } else if (button2X % 2 == 0) {
                    autorot = true;
                    targrot = false;
                    telemetry.addLine("autorot");
                } else {
                    targrot = false;
                    autorot = false;
                    telemetry.addLine("olddddddddddddddddddddddddddddddddddddddddddddd");
                }
            }

            if (gamepad2.y && !but2Ycheck) {
                button2Y += 1;
                but2Ycheck = true;
            }

            if (!gamepad2.y) {
                but2Ycheck = false;
            }
            if (autorot) {
                if (but2Ycheck) {
                    if (button2Y % 2 == 1) {
                        rottarg = getAngle() - oroffset - 180;
                    } else {
                        rottarg = getAngle() - oroffset;
                    }
                }
                rot = (clamp(rotpower, -1, 1));
            } else if (targrot){
                rottarg = getAngle() - oroffset - contangle;
                rot = (clamp(rotpower, -1, 1));
            } else {
                rot = gamepad1.left_stick_x;
            }

 */

            offset = (fieldangle() - imureset);
            oroffset = (getAngle()%360) - imureset;

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

            detections = aprilTagProcessor.getDetections();


            if (detections.size() > 0) {
                telemetry.addData("Tags Detected", detections.size());

                for (AprilTagDetection tag : detections) {
                    int tagIdCode = tag.id;
                    telemetry.addData("Tag ID", tagIdCode);

                    // Calculate vector from center (320, 240)
                    double tagX = tag.center.x;
                    double tagY = tag.center.y;
                    double dx = tagX - 320;
                    double dy = tagY - 240;
                    double pixelDistance = Math.sqrt(dx * dx + dy * dy);

                    telemetry.addData("Tag Center (pixels)", String.format("(%.1f, %.1f)", tagX, tagY));
                    telemetry.addData("Vector from Center", String.format("<%.1f, %.1f>", dx, dy));
                    telemetry.addData("Magnitude of Vector", String.format("%.2f pixels", pixelDistance));

                    if (tag.metadata != null) {
                        String tagName = tag.metadata.name;
                        telemetry.addData("Tag Name", tagName);

                        double range = tag.ftcPose.range;
                        double bearing = tag.ftcPose.bearing;
                        double elevation = tag.ftcPose.elevation;
                        telemetry.addLine("range " + (range + 4));
                        telemetry.addLine("bearing " + bearing);
                        telemetry.addLine("elevation " + elevation);
                    } else {
                        telemetry.addData("Tag Name", "Not in Library");
                        telemetry.addLine("Pose Data Unavailable (Missing Metadata)");
                    }

                    // Custom tag messages
                    if (tagIdCode == 21) {
                        telemetry.addLine("Tag 21 detected: Green Purple Purple.");
                    } else if (tagIdCode == 22) {
                        telemetry.addLine("Tag 22 detected: Purple Green Purple.");
                    } else if (tagIdCode == 23) {
                        telemetry.addLine("Tag 23 detected: Purple Purple Green.");
                    }
                }
            } else {
                telemetry.addLine("Tags Detected: None");
            }

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


    private void initAprilTag() {
        AprilTagLibrary.Builder myAprilTagLibraryBuilder = new AprilTagLibrary.Builder();

        // Add default tags
        myAprilTagLibraryBuilder.addTags(AprilTagGameDatabase.getCurrentGameTagLibrary());

        myAprilTagLibraryBuilder.setAllowOverwrite(true);

        // Add custom tags
        myAprilTagLibraryBuilder.addTag(21, "Left Tag", 4.0, DistanceUnit.INCH);
        myAprilTagLibraryBuilder.addTag(22, "Center Tag", 4.0, DistanceUnit.INCH);
        myAprilTagLibraryBuilder.addTag(23, "Right Tag", 4.0, DistanceUnit.INCH);

        // Build the final tag library
        AprilTagLibrary myAprilTagLibrary = myAprilTagLibraryBuilder.build();

        // Create AprilTagProcessor using Builder and custom Library
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setTagLibrary(myAprilTagLibrary)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .build();

        // Create VisionPortal using Builder chain
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "testcam"))
                .addProcessor(aprilTagProcessor)
                .setCameraResolution(new Size(640, 480)) // Camera resolution
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .setAutoStopLiveView(true)
                .build();

        telemetry.addLine("Status: VisionPortal and AprilTagProcessor Initialized");
        telemetry.update();

        visionPortal.setProcessorEnabled(aprilTagProcessor, true);
    }

    /*public List<AprilTagDetection> detectAprilTags() {
        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();

        telemetry.addLine("");
        telemetry.addLine("=== AprilTag Detection ===");

        if (detections.size() > 0) {
            telemetry.addData("Tags Detected", detections.size());

            for (AprilTagDetection tag : detections) {
                int tagIdCode = tag.id;
                telemetry.addData("Tag ID", tagIdCode);

                // Calculate vector from center (320, 240)
                double tagX = tag.center.x;
                double tagY = tag.center.y;
                double dx = tagX - 320;
                double dy = tagY - 240;
                double pixelDistance = Math.sqrt(dx * dx + dy * dy);

                telemetry.addData("Tag Center (pixels)", String.format("(%.1f, %.1f)", tagX, tagY));
                telemetry.addData("Vector from Center", String.format("<%.1f, %.1f>", dx, dy));
                telemetry.addData("Magnitude of Vector", String.format("%.2f pixels", pixelDistance));

                if (tag.metadata != null) {
                    String tagName = tag.metadata.name;
                    telemetry.addData("Tag Name", tagName);

                    double range = tag.ftcPose.range;
                    double bearing = tag.ftcPose.bearing;
                    double elevation = tag.ftcPose.elevation;
                    telemetry.addLine("range " + (range + 4));
                    telemetry.addLine("bearing " + bearing);
                    telemetry.addLine("elevation " + elevation);
                } else {
                    telemetry.addData("Tag Name", "Not in Library");
                    telemetry.addLine("Pose Data Unavailable (Missing Metadata)");
                }

                // Custom tag messages
                if (tagIdCode == 21) {
                    telemetry.addLine("Tag 21 detected: Green Purple Purple.");
                } else if (tagIdCode == 22) {
                    telemetry.addLine("Tag 22 detected: Purple Green Purple.");
                } else if (tagIdCode == 23) {
                    telemetry.addLine("Tag 23 detected: Purple Purple Green.");
                }
            }
        } else {
            telemetry.addLine("Tags Detected: None");
        }
        return detections;
        }
     */

}