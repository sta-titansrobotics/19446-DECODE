package movement;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp
public class coords_auto extends LinearOpMode {

    // Button tracking variables
    int button2X = 0;
    int button2Y = 0;
    boolean but2Xcheck = false;
    boolean but2Ycheck = false;

    // Direction and magnitude for movement
    static double dir;
    static double mag;
    static double pi = Math.PI;

    // Rotation control variables
    double rottarg = 0;
    double roterr;
    double rotpower;
    double rotpreverr = 0;
    double totroterr;

    // PD constants for rotation
    double rotkp = 0.01;
    double rotkd = 0.01;

    double rot;

    // Offset tracking
    double offset = 0;
    double oroffset = 0;
    double imureset = 0;

    // IMU variables
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    Orientation lastAngles1 = new Orientation();
    double angle = 0;
    double angle1 = 0;
    boolean autorot = false;

    // Vision variables
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;

    // AprilTag auto-rotation variables
    boolean tagDetectedPreviously = false;
    boolean autoRotating = false;
    double autoRotationTarget = 0;

    @Override
    public void runOpMode() {

        // ----------------------Set Up------------------------------------------------
        // Motor initialization
        DcMotor FL = hardwareMap.get(DcMotor.class, "FL");
        DcMotor BL = hardwareMap.get(DcMotor.class, "BL");
        DcMotor FR = hardwareMap.get(DcMotor.class, "FR");
        DcMotor BR = hardwareMap.get(DcMotor.class, "BR");

        // IMU setup
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Motor directions
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set brake behavior
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize AprilTag detection
        initAprilTag();

        telemetry.addLine("Robot initialized. Waiting for start...");
        telemetry.addLine("AprilTag auto-rotation enabled!");
        telemetry.update();

        waitForStart();

        if (isStopRequested())
            return;

        while (opModeIsActive()) {
            // ------------------DRIVE TRAIN---------------------------------

            // Reset IMU offset if all gamepad2 buttons pressed
            if (gamepad2.x && gamepad2.a && gamepad2.b && gamepad2.y) {
                imureset = fieldangle();
                telemetry.addLine("IMU Reset!");
            }

            // Calculate rotation error and power
            totroterr = rottarg - (getAngle() % 360);
            roterr = totroterr - 360.0 * Math.floor((totroterr + 180.0) / 360.0);
            rotpower = -(roterr * rotkp) + ((roterr - rotpreverr) * rotkd);
            rotpreverr = roterr;

            // Toggle autorotation with gamepad2.x
            if (gamepad2.x && !but2Xcheck) {
                button2X += 1;
                but2Xcheck = true;
            }
            if (!gamepad2.x) {
                but2Xcheck = false;
            }
            if (but2Xcheck) {
                autorot = (button2X % 2 == 0);
            }

            // Set rotation target with gamepad2.y
            if (gamepad2.y && !but2Ycheck) {
                button2Y += 1;
                but2Ycheck = true;
            }
            if (!gamepad2.y) {
                but2Ycheck = false;
            }

            // Detect AprilTags FIRST - before handling rotation
            boolean tagDetectedNow = detectAprilTags();

            // Trigger 180 rotation when tag is first detected
            if (tagDetectedNow && !tagDetectedPreviously) {
                autorot = true;
                autoRotating = true;
                autoRotationTarget = getAngle() + 180;
                telemetry.addLine("*** TAG DETECTED - ROTATING 180° ***");
            }

            tagDetectedPreviously = tagDetectedNow;

            // Handle rotation mode
            if (autoRotating) {
                // Override rotation control when auto-rotating from tag detection
                autorot = true;
                rottarg = autoRotationTarget;
                rot = clamp(rotpower, -1, 1);

                // Check if rotation is complete (within 5 degrees)
                if (Math.abs(roterr) < 5) {
                    autoRotating = false;
                    telemetry.addLine("*** ROTATION COMPLETE ***");
                }
            } else if (autorot) {
                // Normal auto-rotation mode (gamepad controlled)
                if (but2Ycheck) {
                    if (button2Y % 2 == 1) {
                        rottarg = getAngle() - oroffset - 180;
                    } else {
                        rottarg = getAngle() - oroffset;
                    }
                }
                rot = clamp(rotpower, -1, 1);
            } else {
                // Manual rotation control
                rot = gamepad1.left_stick_x;
            }

            // Update offsets
            offset = (fieldangle() - imureset);
            oroffset = (getAngle() % 360) - imureset;

            // Calculate movement direction and magnitude
            dir = Math.atan2(-gamepad1.right_stick_y, gamepad1.right_stick_x) - offset;
            mag = Math.sqrt(Math.pow(gamepad1.right_stick_x, 2) + Math.pow(gamepad1.right_stick_y, 2));
            mag *= Math.sqrt(2);
            if (mag > Math.sqrt(2))
                mag = Math.sqrt(2);

            // Slow mode with button B
            if (gamepad1.b) {
                rot *= 0.5;
                mag *= 0.5;
            }

            // Set motor powers
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

            // Telemetry
            telemetry.addLine("=== Drivetrain 19446 ===");
            telemetry.addLine("");
            telemetry.addData("FR Power", "%.2f", FR.getPower());
            telemetry.addData("FL Power", "%.2f", FL.getPower());
            telemetry.addData("BR Power", "%.2f", BR.getPower());
            telemetry.addData("BL Power", "%.2f", BL.getPower());
            telemetry.addLine("");
            telemetry.addData("Manual Auto Rotation", autorot ? "ON" : "OFF");
            telemetry.addData("Tag Auto-Rotate", autoRotating ? "ACTIVE <<<" : "Idle");
            telemetry.addData("Rotation Target", "%.2f", rottarg);
            telemetry.addData("Rotation Error", "%.2f", roterr);
            telemetry.addData("Rotation Power", "%.2f", rotpower);
            telemetry.addData("Current Angle", "%.2f", getAngle());
            telemetry.addData("Field Angle", "%.2f", fieldangle());
            telemetry.addData("Offset", "%.2f", offset);
            telemetry.update();
        }

        // Clean up vision portal
        if (visionPortal != null) {
            visionPortal.close();
        }
    }

    /**
     * Get the current angle from IMU (0-360 continuous)
     */
    private double getAngle() {
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

    /**
     * Get field-relative angle in radians
     */
    private double fieldangle() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

        double deltaAngle = angles.firstAngle - lastAngles1.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        angle1 += deltaAngle;
        lastAngles1 = angles;

        return angle1;
    }

    /**
     * Clamp a value between min and max
     */
    public double clamp(double value, double min, double max) {
        return Math.min(Math.max(value, min), max);
    }

    /**
     * Initialize AprilTag detection system
     */
    private void initAprilTag() {
        // Build custom tag library
        AprilTagLibrary.Builder tagBuilder = new AprilTagLibrary.Builder();
        tagBuilder.addTags(AprilTagGameDatabase.getCurrentGameTagLibrary());
        tagBuilder.setAllowOverwrite(true);
        tagBuilder.addTag(21, "Left Tag", 4.0, DistanceUnit.INCH);
        tagBuilder.addTag(22, "Center Tag", 4.0, DistanceUnit.INCH);
        tagBuilder.addTag(23, "Right Tag", 4.0, DistanceUnit.INCH);

        AprilTagLibrary tagLibrary = tagBuilder.build();

        // Create AprilTag processor
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setTagLibrary(tagLibrary)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .build();

        // Build vision portal - FIXED: Removed setAutoStopLiveView
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "testcam"))
                .addProcessor(aprilTagProcessor)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .build();

        // Enable the processor
        visionPortal.setProcessorEnabled(aprilTagProcessor, true);
    }

    /**
     * Detect and display AprilTag information
     * @return true if any tags are detected, false otherwise
     */
    private boolean detectAprilTags() {
        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();

        telemetry.addLine("");
        telemetry.addLine("=== AprilTag Detection ===");

        if (detections.size() > 0) {
            telemetry.addData("Tags Detected", detections.size());
            telemetry.addLine("");

            for (AprilTagDetection tag : detections) {
                telemetry.addData("Tag ID", tag.id);

                if (tag.metadata != null) {
                    telemetry.addData("  Name", tag.metadata.name);
                    telemetry.addData("  Range", "%.1f inches", tag.ftcPose.range);
                    telemetry.addData("  Bearing", "%.1f degrees", tag.ftcPose.bearing);
                    telemetry.addData("  Elevation", "%.1f degrees", tag.ftcPose.elevation);
                    telemetry.addData("  X Position", "%.1f inches", tag.ftcPose.x);
                    telemetry.addData("  Y Position", "%.1f inches", tag.ftcPose.y);
                    telemetry.addData("  Z Position", "%.1f inches", tag.ftcPose.z);
                    telemetry.addData("  Yaw", "%.1f degrees", tag.ftcPose.yaw);
                    telemetry.addData("  Pitch", "%.1f degrees", tag.ftcPose.pitch);
                    telemetry.addData("  Roll", "%.1f degrees", tag.ftcPose.roll);
                } else {
                    telemetry.addData("  Name", "Unknown");
                }

                // Custom tag messages
                switch (tag.id) {
                    case 21:
                        telemetry.addData("  Pattern", "Green Purple Purple");
                        break;
                    case 22:
                        telemetry.addData("  Pattern", "Purple Green Purple");
                        break;
                    case 23:
                        telemetry.addData("  Pattern", "Purple Purple Green");
                        break;
                }

                telemetry.addLine("");
            }

            return true; // Tags detected
        } else {
            telemetry.addLine("No tags detected");
            return false; // No tags detected
        }
    }
}