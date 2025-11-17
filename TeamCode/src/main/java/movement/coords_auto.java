package movement;

import android.util.Size;

import androidx.core.graphics.ColorUtils;

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

@TeleOp
public class coords_auto extends LinearOpMode {

    int button2X = 0;
    int button2A = 0;
    int button2B = 0;
    int button2Y = 0;
    int buttonA = 0;
    int buttonB = 0;
    int buttonX = 0;
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

    double torquetarg;

    double rotkp = 0.01;
    double rotkd = 0.01;

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
    double contangle;

    double velo;
    double prevpos;
    double prevtime1;

    double shootp;



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

        CRServo tubes = hardwareMap.get(CRServo.class, "tubes");
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

            //will cause the offset to be set back to 0
            if (gamepad1.left_trigger>0.8&&gamepad1.right_trigger>0.8)
                imureset = fieldangle();

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

            if (velo > 1200){
//                    if (getRuntime() - prevtime > 2000)
//                        prevtime = getRuntime();
//                    else if (getRuntime() - prevtime > 1000)
//                        elev.setPower(gamepad2.right_trigger);
//                    else if (getRuntime() - prevtime < 1000)
//                        elev.setPower(0);
                elev.setPower(gamepad2.right_trigger);
            } else {
                if (sensorColor.alpha()<75||sensorDistance.getDistance(DistanceUnit.CM)>4.5){
                    elev.setPower(gamepad2.right_trigger);
                } else {
                    elev.setPower(0);
                }
            }


            tubes.setPower(-gamepad2.right_trigger);

            //shoot.setPower(gamepad1.left_trigger);
            //if (gamepad2.b)
            //    angles.setPosition(gamepad2.left_trigger);


            if (gamepad2.dpad_up) {
                shootp = 0.64;
                angles.setPosition(0.5);
            }
            else if (gamepad2.dpad_right) {
                shootp = 0.82;
                angles.setPosition(0.5);
            }
            else if (gamepad2.dpad_left) {
                shootp = 0.85;
                angles.setPosition(0.5);
            } else if (gamepad2.dpad_down) {
                shootp = 0;
                angles.setPosition(0);
            }
            if (shootp==0.64) {
                if (velo < 1200)
                    shoot.setPower(shootp);
                else if (velo > 1300)
                    shoot.setPower(0);
            } else if (shootp==0.85) {
                if (velo < 1500)
                    shoot.setPower(shootp);
                else if (velo > 1550)
                    shoot.setPower(0);
            }else if (shootp == 0.82){
                if (velo < 1500)
                    shoot.setPower(shootp);
                else if (velo > 1450)
                    shoot.setPower(0);
            }else if (shootp==0){
                shoot.setPower(0);
            }

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
            prevtime1 = getRuntime();
            prevpos = shoot.getCurrentPosition();
            telemetry.addData("velo", velo);
            telemetry.addData("pos", shoot.getCurrentPosition());

            if (velo >1200){
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


            if (gamepad1.x && !but2Xcheck) {
                buttonX += 1;
                if (buttonX > 3)
                    buttonX = 1;
                butXcheck = true;
            }

            if (!gamepad1.x) {
                butXcheck = false;
            }

            if (butXcheck) {
                if (buttonX % 3 == 0) {
                    autorot = false;
                    targrot = true;
                } else if (buttonX % 2 == 0) {
                    autorot = true;
                    targrot = false;
                } else {
                    targrot = false;
                    autorot = false;
                }
            }
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
                contangle += 5*gamepad1.left_stick_x;

                if (gamepad1.left_stick_x !=0){
                    rot = gamepad1.left_stick_x;
                    contangle = (getAngle()%360);
                }
                rottarg = getAngle() - oroffset - contangle;
            }

            if (gamepad1.a && !butAcheck){
                buttonA += 1;
                butAcheck = true;
            }

            if (!gamepad1.a){
                butAcheck = false;
            }

            if (!butAcheck) {
                if (buttonA % 2 == 1) {
                    intake.setPower(1);
                } else {
                    intake.setPower(0);
                }
            }

            //imu increases when turning left and decreases when turning right

            //the offset variable tells it how much it has deviated from the original orientation so that it can still move in the correct direction when rotated

            //the dir variable is the variable that determines where we want to be on the sine wave

            totroterr = rottarg - (getAngle() % 360);

            roterr = totroterr - 360.0 * Math.floor((totroterr + 180.0) / 360.0);

            // actual pd calculations
            rotpower = -(roterr * rotkp) + ((roterr - rotpreverr) * rotkd);

            // getting the previous error
            rotpreverr = roterr;

            if (autorot) {
                rot = (clamp(rotpower, -1, 1));
            } else if (targrot){
                if (gamepad1.left_stick_x !=0)
                    rot = gamepad1.left_stick_x;
                else
                    rot = (clamp(rotpower, -1, 1));
            } else {
                rot = gamepad1.left_stick_x;
            }

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

            List<AprilTagDetection> detections = aprilTagProcessor.getDetections();

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