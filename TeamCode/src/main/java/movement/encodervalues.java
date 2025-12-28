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
import com.acmerobotics.dashboard.FtcDashboard;
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
public class encodervalues extends LinearOpMode {

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

    double rawpodx;
    double rawpody;

    double podx;
    double pody;

    double erry;
    double errx;

    double targy;
    double powery;
    double preverry;

    double targx;
    double powerx;
    double preverrx;

    double xkp = 0.01;
    double xkd = 0.01;

    double ykp = 0.01;
    double ykd = 0.01;

    double tpr = 30;

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

        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elev.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        gled.enable(false);
        rled.enable(false);
        gled1.enable(false);
        rled1.enable(false);


        waitForStart();

        if (isStopRequested())
            return;

        FtcDashboard dashboard = FtcDashboard.getInstance();

        while (opModeIsActive()) {

            offset = (fieldangle() - imureset);
            oroffset = (getAngle()%360) - imureset;

            rawpody = FL.getCurrentPosition(); // FL motor is being treated as the y odometry pod - measuring the froward/back motion - where the Y pod is right now
            rawpodx = BL.getCurrentPosition(); // BL motor is being treated as the x odometry pod - measures the left/right strafe - where the X pod is right now

            rottarg = getAngle() - oroffset;

            //imu increases when turning left and decreases when turning right

            //the offset variable tells it how much it has deviated from the original orientation so that it can still move in the correct direction when rotated

            //the dir variable is the variable that determines where we want to be on the sine wave

            totroterr = rottarg - (getAngle() % 360);

            roterr = totroterr - 360.0 * Math.floor((totroterr + 180.0) / 360.0);

            // actual pd calculations
            rotpower = -(roterr * rotkp) + ((roterr - rotpreverr) * rotkd);

            // getting the previous error
            rotpreverr = roterr;


            // ododododododododod mentereyrereyreyrasdlkfjad;fldksj odometry



            podx = rawpodx-(getAngle()*tpr);
            pody = rawpody-(getAngle()*tpr);


            //targx = 5000;
            //targy = 5000;

            errx = targx - rawpodx;
            erry = targy - rawpody;

            // actual pd calculations
            powerx = errx*xkp+(errx - preverrx)*xkd;
            powery = erry*ykp+(erry - preverry)*ykd;

            powerx = clamp(powerx, 0, 1);
            powery = clamp(powery, 0, 1);

            // getting the previous error
            preverrx = (targx - rawpodx);
            preverry = (targy - rawpody);


            // ------------------DRIVE TRAIN---------------------------------

            dashboard.getTelemetry().addLine("olddddddddddddddddddddddddddddddddddddddddddddd");
            dir = Math.atan2(erry, errx) - offset;
            mag = Math.sqrt(Math.pow(powerx, 2) + Math.pow(powery, 2));
            mag *= sqrt2;
            if (mag > sqrt2)
                mag = sqrt2;


            //CHANGE ROTATION TO BE CONTROLLED BY GAMEPAD 2
            if (gamepad1.right_trigger>0.1)
                rot *= 0.5;
            if (gamepad1.right_trigger>0.1)
                mag *= 0.5;


            dashboard.getTelemetry().addLine("");
            dashboard.getTelemetry().addLine("");
            dashboard.getTelemetry().addLine("");

            dashboard.getTelemetry().addData("FL", FL.getCurrentPosition());
            dashboard.getTelemetry().addData("BL", BL.getCurrentPosition());
            dashboard.getTelemetry().addData("FR", FR.getCurrentPosition());
            dashboard.getTelemetry().addData("BR", BR.getCurrentPosition());
            
            dashboard.getTelemetry().addData("rawpodx", rawpodx);
            dashboard.getTelemetry().addData("rawpody", rawpody);
            dashboard.getTelemetry().addData("podx", podx);
            dashboard.getTelemetry().addData("pody", pody);
            dashboard.getTelemetry().addData("errx", errx);
            dashboard.getTelemetry().addData("erry", erry);
            dashboard.getTelemetry().addData("powerx", powerx);
            dashboard.getTelemetry().addData("powery", powery);
            dashboard.getTelemetry().addData("preverrx", preverrx);
            dashboard.getTelemetry().addData("preverry", preverry);
            dashboard.getTelemetry().addData("rotpower", rotpower);
            dashboard.getTelemetry().addData("rotpreverr", rotpreverr);

            dashboard.getTelemetry().addLine("");
            dashboard.getTelemetry().addLine("");
            dashboard.getTelemetry().addLine("");

            dashboard.getTelemetry().addData("rot", rot);
            dashboard.getTelemetry().addData("mag", mag);
            dashboard.getTelemetry().addData("dir", dir);



            dashboard.getTelemetry().addLine("");
            dashboard.getTelemetry().addLine("<------------------->");
            dashboard.getTelemetry().addLine("");

            //dashboard.getTelemetry().addData("", );

            //-----------------apriltagsssssssss------------------------------------

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
}