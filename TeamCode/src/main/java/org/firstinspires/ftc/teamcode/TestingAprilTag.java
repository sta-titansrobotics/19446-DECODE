package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;

import android.util.Size;

import java.util.List;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "AprilTag Full Init Demo", group = "Vision")
public class TestingAprilTag extends LinearOpMode {

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;

    @Override
    public void runOpMode() {

        //custom AprilTagLibrary
        AprilTagLibrary.Builder myAprilTagLibraryBuilder = new AprilTagLibrary.Builder();

        // Adding default tags
        myAprilTagLibraryBuilder.addTags(AprilTagGameDatabase.getCurrentGameTagLibrary());

        myAprilTagLibraryBuilder.setAllowOverwrite(true);

        // Add custom tags
        myAprilTagLibraryBuilder.addTag(21, "Left Tag", 4.0, DistanceUnit.INCH);
        myAprilTagLibraryBuilder.addTag(22, "Center Tag", 4.0, DistanceUnit.INCH);
        myAprilTagLibraryBuilder.addTag(23, "Right Tag", 4.0, DistanceUnit.INCH);

        //Build the final tag library
        AprilTagLibrary myAprilTagLibrary = myAprilTagLibraryBuilder.build();

        //Create AprilTagProcessor using Builder and custom Library
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setTagLibrary(myAprilTagLibrary)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .build();

        //Create VisionPortal using Builder chain
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "testcam"))
                .addProcessor(aprilTagProcessor)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .setAutoStopLiveView(true)
                .build();

        telemetry.addLine("Status: VisionPortal and AprilTagProcessor Initialized");
        telemetry.update();

        visionPortal.setProcessorEnabled(aprilTagProcessor, true);

        waitForStart();

        while (opModeIsActive()) {
            List<AprilTagDetection> detections = aprilTagProcessor.getDetections();

            if (detections.size() > 0) {
                telemetry.addData("Tags Detected", detections.size());

                for (AprilTagDetection tag : detections) {
                    int tagIdCode = tag.id;
                    telemetry.addData("Tag ID", tagIdCode);

                    if (tag.metadata != null) {
                        String tagName = tag.metadata.name;
                        telemetry.addData("Tag Name", tagName);

                        double range = tag.ftcPose.range;
                        double bearing = tag.ftcPose.bearing;
                        double elevation = tag.ftcPose.elevation;
                        telemetry.addLine("range " + range);
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
            sleep(100);
        }

        // VisionPortal shuts down automatically when OpMode ends
    }
}
