package movement;

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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

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

    // HELOLOSOSDOJIASDJKFA;SDLK

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

    @Override
    public void runOpMode() {

        // ----------------------Set Up------------------------------------------------
        // Moving
        DcMotor FL = hardwareMap.get(DcMotor.class, "FL"); // Expansion hub
        DcMotor BL = hardwareMap.get(DcMotor.class, "BL"); // Expansion hub
        DcMotor FR = hardwareMap.get(DcMotor.class, "FR"); // Expantion hub
        DcMotor BR = hardwareMap.get(DcMotor.class, "BR"); // Expantion hub

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



        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);

        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



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

            totroterr = rottarg - (getAngle()%360);

            roterr = totroterr - 360.0 * Math.floor((totroterr + 180.0) / 360.0);

            // actual pd calculations
            rotpower = -(roterr*rotkp)+((roterr - rotpreverr)*rotkd);

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

            if (gamepad2.x && !but2Xcheck) {
                button2X += 1;
                but2Xcheck = true;
            }

            if (!gamepad2.x) {
                but2Xcheck = false;
            }

            if (but2Xcheck) {
                if (button2X % 2 == 1) {
                    autorot = false;
                } else {
                    autorot = true;
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
            } else {
                rot = gamepad1.left_stick_x;
            }

            offset = (fieldangle() - imureset);
            oroffset = (getAngle()%360) - imureset;

            //imu increases when turning left and decreases when turning right

            //the offset variable tells it how much it has deviated from the original orientation so that it can still move in the correct direction when rotated

            //the dir variable is the variable that determines where we want to be on the sine wave

            dir = Math.atan2(-gamepad1.right_stick_y, gamepad1.right_stick_x)-offset;
            mag = Math.sqrt(Math.pow(gamepad1.right_stick_x, 2) + Math.pow(gamepad1.right_stick_y, 2));
            mag *= Math.sqrt(2);
            if (mag > Math.sqrt(2))
                mag = Math.sqrt(2);





            //CHANGE ROTATION TO BE CONTROLLED BY GAMEPAD 2
            if (gamepad1.b)
                rot *= 0.5;
            if (gamepad1.b)
                mag *= 0.5;


            if (gamepad1.right_stick_y != 0 || gamepad1.right_stick_x != 0 || gamepad1.left_stick_x != 0 || rot != 0){
                FR.setPower((Math.sin(dir-(pi/4))*mag) - rot);
                FL.setPower((Math.sin(dir+(pi/4))*mag) + rot);
                BR.setPower((Math.sin(dir+(pi/4))*mag) - rot);
                BL.setPower((Math.sin(dir-(pi/4))*mag) + rot);
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


            //telemetry.addData("", );

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

    private double fieldangle()
    {

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
    public double clamp(double value, double min, double max) {
        return Math.min(Math.max(value, min), max);
    }
}
