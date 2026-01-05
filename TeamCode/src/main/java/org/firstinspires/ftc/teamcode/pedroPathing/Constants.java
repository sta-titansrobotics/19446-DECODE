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
import com.pedropathing.paths.PathConstraints;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import movement.tuning;


public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .useSecondaryDrivePIDF(true)
            .useSecondaryTranslationalPIDF(true)
            .useSecondaryHeadingPIDF(true)
            .forwardZeroPowerAcceleration(-27.0695)
            .lateralZeroPowerAcceleration(-78.0674)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.05, 0.00001,0.002,0.01))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.1,0,0.009,0.02))
            .headingPIDFCoefficients(new PIDFCoefficients(2, 0, 0, 0))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(0.4,0,0.01,0.01))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.015,0.0,0.0001,0.6,0.0))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.005,0,0.00005,0.6,0.01))
            .centripetalScaling(0.00085)
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
            .xVelocity(81.0785)
            .yVelocity(61.8035)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);

    public static ThreeWheelIMUConstants threewheelimu = new ThreeWheelIMUConstants()
            .forwardTicksToInches(0.001965)
            .strafeTicksToInches(0.001962)
            .turnTicksToInches(0.001948) //~~~~~~~~~~~~~~~~    CHANGE
            .leftPodY(3.27) // ~~~~~~~~~~~~~~ CHANGE
            .rightPodY(-3.27)
            .strafePodX(5)
            .leftEncoder_HardwareMapName("BR") // ~~~~~~~~~~~~~~ CHANGE
            .rightEncoder_HardwareMapName("FL")
            .strafeEncoder_HardwareMapName("FR")
            .leftEncoderDirection(Encoder.REVERSE) // ~~~~~~~~~~~~~~ CHANGE
            .rightEncoderDirection(Encoder.REVERSE)
            .IMU_HardwareMapName("imu")
            .IMU_Orientation(
                    new RevHubOrientationOnRobot(
                            RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                            RevHubOrientationOnRobot.UsbFacingDirection.UP
                    )
            );

    public static ThreeWheelConstants threewheel = new ThreeWheelConstants()
            .forwardTicksToInches(0.001965)
            .strafeTicksToInches(0.001962)
            .turnTicksToInches(0.001965) //~~~~~~~~~~~~~~~~    CHANGE
            .leftPodY(3.27) // ~~~~~~~~~~~~~~ CHANGE
            .rightPodY(-3.27)
            .strafePodX(5)
            .leftEncoder_HardwareMapName("BR") // ~~~~~~~~~~~~~~ CHANGE
            .rightEncoder_HardwareMapName("FL")
            .strafeEncoder_HardwareMapName("FR")
            .leftEncoderDirection(Encoder.REVERSE) // ~~~~~~~~~~~~~~ CHANGE
            .rightEncoderDirection(Encoder.REVERSE);


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


    public static int localizer = 3;


    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


    public static Follower createFollower(HardwareMap hardwareMap) {
        FollowerBuilder builder = new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants);

        if (localizer == 1) {
            builder.twoWheelLocalizer(localizerConstants);
        } else if (localizer == 2) {
            builder.threeWheelLocalizer(threewheel);
        } else if (localizer == 3) {
            builder.threeWheelIMULocalizer(threewheelimu);
        }

        return builder.build();
    }
}
