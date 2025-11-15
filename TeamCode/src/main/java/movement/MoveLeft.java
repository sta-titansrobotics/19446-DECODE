package movement;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name = "Auto_Left_Odometry", group = "movement")
public class MoveLeft extends LinearOpMode {

    private DcMotor FL, BL, FR, BR;
    Orientation previousAngle = new Orientation();
    double angle = 0.0;
    static final double pi = Math.PI;

    int previousXPod = 0; // previous BR motor encoder reading
    int yPodPrev = 0; // previous FR motor encoder reading
    int xPodPrev = 0;

    double feildXCoord = 0.0;
    double feildYCoord = 0.0;

    static final double xEncoderTicksToActualValue = 1.0;
    static final double yEncoderTicksToActualValue = 1.0;

    static final double KpStrafe = 0.05; // to increase the speed of the robot when strafing

    BNO055IMU imu;

    @Override
    public void runOpMode(){

        // MOTOR SET UP
        FL = hardwareMap.get(DcMotor.class, "FL"); // Expansion hub
        BL = hardwareMap.get(DcMotor.class, "BL"); // Expansion hub
        FR = hardwareMap.get(DcMotor.class, "FR"); // Expantion hub
        BR = hardwareMap.get(DcMotor.class, "BR"); // Expantion hub

        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);

        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // IMU SET UP
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Reset encoder values
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        yPodPrev = FL.getCurrentPosition(); // FR motor is being treated as the y odometry pod - measuring the froward/back motion - where the Y pod is right now
        xPodPrev = BL.getCurrentPosition(); // Br motor is being treated as the x odometry pod - measures the left/right strafe - where the X pod is right now

        // starts at 0,0 for auto...from wherever it is placed (the origin)
        feildXCoord = 0.0;
        feildYCoord = 0.0;

        // Send message to the driver Hub
        telemetry.addLine("Auto Left (Odometry) - READY");
        telemetry.update();

        waitForStart(); // makes the driver have to press start on the driver hub

        if (isStopRequested()){
            return;
        }

        // Move robot to the left
        double targetX = -36.0; // how far robot should move in inches

        while (opModeIsActive()){

            // Update odometry position from enoders and the IMU
            updateOdometry();

            // Calculate how far robot needs to travel in the x
            double errorX = targetX - feildXCoord;

            // tell the robot to stop within 0.5 inches of the target
            if (Math.abs(errorX) < 0.5){
                stopDrive();
                break;
            }

            // P control to set the power when strafing - the current error
            double strafeCmd = KpStrafe * errorX; // product will shrink or grow depending on how big the error is - how fast the robot will strafe depending on how far the robot is from the target
            strafeCmd = Range.clip(strafeCmd, -0.5, 0.5); // Motor power must always be set between -1 and 1 so here is setting the clamp values -

            // Make sure robot is pointing froward
            double heading = getAngle();
            double targetHeading = 0.0; // want it to be this way
            double errorHeading = targetHeading - heading; // how far off the robot's orientation is from where it should be
            double KpHeading = 0.5; // how fast the robot should correct
            double rotCmd = Range.clip(KpHeading * errorHeading, -0.3, 0.3); // clamps the values

            double forwardCmd  = 0.0; // only strafes

            driveRobotCentric(forwardCmd, strafeCmd, rotCmd);

            // Display things on Driver Hub
            telemetry.addLine("Moving LEFT with odometry");
            telemetry.addData("feildXCoord (inches)", feildXCoord);
            telemetry.addData("feildYCoordinate (inches)", feildYCoord);
            telemetry.addData("errorX (inches)", errorX);
            telemetry.addData("heading (rad)", heading);
            telemetry.update();

        }

        sleep(1000);

    }

    private void updateOdometry(){

        // Read encoder values for current position
        int yPod = FR.getCurrentPosition();
        int xPod = BR.getCurrentPosition();

        // changes since last loop
        int changeYTicks = yPod - yPodPrev;
        int changeXTicks = xPod - xPodPrev;

        // convert ticks to inches
        double dyRobot = changeYTicks * xEncoderTicksToActualValue;
        double dxRobot = changeXTicks * yEncoderTicksToActualValue;

        yPodPrev = yPod;
        xPodPrev = xPod;

        double heading = getAngle(); // gets current angle from imu in radians

        // rotate robot so that it aligns with the feild coordinates
        double distXfeild = dyRobot * Math.cos(heading) + dxRobot * Math.cos(heading + pi/2); // arc angle
        double distYfeild = dyRobot * Math.sin(heading) + dxRobot * Math.sin(heading + pi / 2);

        distXfeild += feildXCoord;
        distYfeild += feildYCoord;

    }

    private void stopDrive(){
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
    }

    private double getAngle(){
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS); // orientation of the robot, rotate around the robots axis, order of rotations (Z, then Y, then Z), make sure angle measurement is in radians

        double changeInAngle = angles.firstAngle - previousAngle.firstAngle; // current heading from the IMU - pervious heading on the = change in angle

        if (changeInAngle < -Math.PI){
            changeInAngle += 2.0 * Math.PI;
        }
        else if (changeInAngle > Math.PI){
            changeInAngle -= 2.0 * Math.PI;
        }

        angle += changeInAngle;
        previousAngle = angles;
        return angle;
    }

    private void driveRobotCentric(double forward, double strafe, double rotate){

        double fl = forward + strafe + rotate;
        double fr = forward - strafe - rotate;
        double bl = forward - strafe + rotate;
        double br = forward + strafe - rotate;

        // make sure the power doesn't get greater than one
        double max = Math.max(1.0, Math.max(Math.abs(fl),Math.max(Math.abs(bl), Math.abs(br))));

        fl /= max;
        fr /= max;
        bl /= max;
        br /= max;

        FL.setPower(fl);
        FR.setPower(fr);
        BL.setPower(bl);
        BR.setPower(br);

    }

}
