package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.DriveEncoderConstants;
import com.pedropathing.ftc.localization.constants.TwoWheelConstants;
import com.pedropathing.ftc.localization.constants.ThreeWheelConstants;
import com.pedropathing.ftc.localization.constants.ThreeWheelIMUConstants;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import movement.tuning;


public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .useSecondaryDrivePIDF(false)
            .useSecondaryTranslationalPIDF(false)
            .useSecondaryHeadingPIDF(false)
            .forwardZeroPowerAcceleration(-26.5695)
            .lateralZeroPowerAcceleration(-70.0674)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.05, 0.00001,0.002,0.01))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.1,0,0.009,0.02))
            .headingPIDFCoefficients(new PIDFCoefficients(0.8, 0, 0, 0))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(0.4,0,0.01,0.01))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.015,0.0,0.0001,0.6,0.0))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.005,0,0.00005,0.6,0.01))
            .centripetalScaling(0.000)//85
            .mass(12);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("FR")
            .rightRearMotorName("BR")
            .leftRearMotorName("BL")
            .leftFrontMotorName("FL")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(78)
            .yVelocity(52.5);

    public static PinpointConstants pinpoint = new PinpointConstants()
            .forwardPodY(-3.27)
            .strafePodX(5)
            .hardwareMapName("pinpoint")
            .distanceUnit(DistanceUnit.INCH)
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);


    public static TwoWheelConstants localizerConstants = new TwoWheelConstants()
            .forwardEncoder_HardwareMapName("FL")
            .strafeEncoder_HardwareMapName("BR")
            .IMU_HardwareMapName("imu")
            .forwardPodY(-3.27)
            .strafePodX(5)
            .forwardEncoderDirection(Encoder.REVERSE)
            .forwardTicksToInches(0.001965)
            .strafeTicksToInches(0.001962)
            .IMU_Orientation(
                    new RevHubOrientationOnRobot(
                            RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                            RevHubOrientationOnRobot.UsbFacingDirection.UP
                    )
            );


    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


    public static int localizer = 2;


    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


    public static Follower createFollower(HardwareMap hardwareMap) {
        FollowerBuilder builder = new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants);

        if (localizer == 1) {
            builder.twoWheelLocalizer(localizerConstants);
        } else if (localizer == 2) {
            builder.pinpointLocalizer(pinpoint);
        }

        return builder.build();
    }
}
