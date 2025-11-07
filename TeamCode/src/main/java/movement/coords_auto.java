package movement;

import android.util.Size;

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

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

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

        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        initAprilTag();

        waitForStart();

        if (isStopRequested())
            return;

        while (opModeIsActive()) {
            // ------------------DRIVE TRAIN---------------------------------

            //will cause the offset to be set back to 0
            if (gamepad2.x && gamepad2.a && gamepad2.b && gamepad2.y)
                imureset = fieldangle();

            // 4 stage sliders
            // limiting the motors movement so that it does not try to over extend the slider

            totroterr = rottarg - (getAngle() % 360);

            roterr = totroterr - 360.0 * Math.floor((totroterr + 180.0) / 360.0);

            // actual pd calculations
            rotpower = -(roterr * rotkp) + ((roterr - rotpreverr) * rotkd);

            // getting the previous error
            rotpreverr = roterr;

            //to fix starting jitter
            /*
            if (getAngle() < 10 && rottarg == 1900){
                rotpower = 0;
            }
            */

            // actually setting the motor power

            //rot = gamepad1.left_stick_x;

            contangle += 5*gamepad1.left_stick_x;


            elev.setPower(gamepad1.right_trigger);

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
                } else if (button2X % 2 == 0) {
                    autorot = true;
                    targrot = false;
                } else {
                    targrot = false;
                    autorot = false;
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

            offset = (fieldangle() - imureset);
            oroffset = (getAngle()%360) - imureset;

            //imu increases when turning left and decreases when turning right

            //the offset variable tells it how much it has deviated from the original orientation so that it can still move in the correct direction when rotated

            //the dir variable is the variable that determines where we want to be on the sine wave

            if (gamepad1.a && !butAcheck){
                buttonA += 1;
                butAcheck = true;
            }

            if (!gamepad1.a){
                butAcheck = false;
            }

            if (!butAcheck){
                if (buttonA % 2 == 1) {
                    intake.setPower(1);
                } else {
                    intake.setPower(0);
                }

                telemetry.addLine("olddddddddddddddddddddddddddddddddddddddddddddd");
                dir = Math.atan2(-gamepad1.right_stick_y, gamepad1.right_stick_x) - offset;
                mag = Math.sqrt(Math.pow(gamepad1.right_stick_x, 2) + Math.pow(gamepad1.right_stick_y, 2));
                mag *= sqrt2;
                if (mag > sqrt2)
                    mag = sqrt2;


                //CHANGE ROTATION TO BE CONTROLLED BY GAMEPAD 2
                if (gamepad1.b)
                    rot *= 0.5;
                if (gamepad1.b)
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