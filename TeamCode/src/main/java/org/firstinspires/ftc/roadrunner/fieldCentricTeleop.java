package org.firstinspires.ftc.roadrunner;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;

import android.util.Size;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="FieldCentricAprilTagOpMode", group="Linear Opmode")
public class fieldCentricTeleop extends LinearOpMode {
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;

    private BNO055IMU imu;

    @Override
    public void runOpMode() {
        // Motor setup
        frontLeftMotor = hardwareMap.get(DcMotor.class, "FL");
        frontRightMotor = hardwareMap.get(DcMotor.class, "FR");
        backLeftMotor = hardwareMap.get(DcMotor.class, "BL");
        backRightMotor = hardwareMap.get(DcMotor.class, "BR");

        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);

        double speedFactor = 0.25;

        // IMU setup
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        initAprilTag(); // Initialize AprilTag system

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double x = -gamepad2.left_stick_y;
            double y = gamepad2.left_stick_x;
            double rx = -gamepad2.right_stick_x;

            double botHeading = getHeading();

            double rotatedX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotatedY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            if (gamepad2.x) {
                rotateToApproximateStraight();
            }

            double frontLeftPower = (rotatedY + rotatedX + rx) * speedFactor;
            double backLeftPower = (rotatedY - rotatedX + rx) * speedFactor;
            double frontRightPower = (rotatedY - rotatedX - rx) * speedFactor;
            double backRightPower = (rotatedY + rotatedX - rx) * speedFactor;

            double maxPower = Math.max(Math.abs(frontLeftPower), Math.abs(backLeftPower));
            maxPower = Math.max(maxPower, Math.abs(frontRightPower));
            maxPower = Math.max(maxPower, Math.abs(backRightPower));

            if (maxPower > 1.0) {
                frontLeftPower /= maxPower;
                backLeftPower /= maxPower;
                frontRightPower /= maxPower;
                backRightPower /= maxPower;
            }

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            detectAprilTags(); // Continuously detect AprilTags

            telemetry.addData("Status", "Running");
            telemetry.addData("Heading (rad)", botHeading);
            telemetry.addData("Front Left Power", frontLeftMotor.getPower());
            telemetry.addData("Front Right Power", frontRightMotor.getPower());
            telemetry.addData("Back Left Power", backLeftMotor.getPower());
            telemetry.addData("Back Right Power", backRightMotor.getPower());
            telemetry.update();
        }
    }

    private void initAprilTag() {
        AprilTagLibrary.Builder tagBuilder = new AprilTagLibrary.Builder();
        tagBuilder.addTags(AprilTagGameDatabase.getCurrentGameTagLibrary());
        tagBuilder.setAllowOverwrite(true);
        tagBuilder.addTag(21, "Left Tag", 4.0, DistanceUnit.INCH);
        tagBuilder.addTag(22, "Center Tag", 4.0, DistanceUnit.INCH);
        tagBuilder.addTag(23, "Right Tag", 4.0, DistanceUnit.INCH);

        AprilTagLibrary tagLibrary = tagBuilder.build();

        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setTagLibrary(tagLibrary)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "testcam"))
                .addProcessor(aprilTagProcessor)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .setAutoStopLiveView(true)
                .build();

        visionPortal.setProcessorEnabled(aprilTagProcessor, true);
    }

    private void detectAprilTags() {
        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();

        if (detections.size() > 0) {
            telemetry.addData("Tags Detected", detections.size());

            for (AprilTagDetection tag : detections) {
                telemetry.addData("Tag ID", tag.id);

                if (tag.metadata != null) {
                    telemetry.addData("Tag Name", tag.metadata.name);
                    telemetry.addData("Range", tag.ftcPose.range);
                    telemetry.addData("Bearing", tag.ftcPose.bearing);
                    telemetry.addData("Elevation", tag.ftcPose.elevation);
                } else {
                    telemetry.addData("Tag Name", "Unknown");
                }

                switch (tag.id) {
                    case 21:
                        telemetry.addLine("Tag 21: Green Purple Purple");
                        break;
                    case 22:
                        telemetry.addLine("Tag 22: Purple Green Purple");
                        break;
                    case 23:
                        telemetry.addLine("Tag 23: Purple Purple Green");
                        break;
                }
            }
        } else {
            telemetry.addLine("Tags Detected: None");
        }
    }

    private void rotateToApproximateStraight() {
        rotateToAngle(0.0);
    }

    private void rotateToAngle(double targetAngle) {
        double currentAngle = getHeading();
        double deltaAngle = normalizeAngle(targetAngle - currentAngle);

        double rotationPower = 0.3;

        if (deltaAngle > 0) {
            setAllMotorPowers(rotationPower, -rotationPower);
        } else {
            setAllMotorPowers(-rotationPower, rotationPower);
        }

        while (Math.abs(deltaAngle) > Math.toRadians(1) && opModeIsActive()) {
            currentAngle = getHeading();
            deltaAngle = normalizeAngle(targetAngle - currentAngle);
        }

        stopAllMotors();
    }

    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    private double getHeading() {
        Orientation angles = imu.getAngularOrientation();
        return angles.firstAngle;
    }

    private void setAllMotorPowers(double leftPower, double rightPower) {
        frontLeftMotor.setPower(leftPower);
        backLeftMotor.setPower(leftPower);
        frontRightMotor.setPower(rightPower);
        backRightMotor.setPower(rightPower);
    }

    private void stopAllMotors() {
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
    }
}
