package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Subsystem to manage the flywheel motor and its velocity PID control.
 */
public class FlywheelSubsystem {
    private DcMotor shoot;
    private Servo angles; // Used for adjusting the shooting angle

    // PID Target and Constants
    private double TARGET_RPM = 0;
    private final double SKP = 0.005;
    private final double SKD = 0.01;

    // PID State Variables
    private double currentVelo = 0;
    private double prevPos = 0;
    private double shooterDeriv = 0;

    // Timer for velocity calculation
    private ElapsedTime timer = new ElapsedTime();

    // Safety Margin for ready-to-shoot check (e.g., within 5% of target)
    private final double VELOCITY_TOLERANCE_PERCENT = 0.05;

    public FlywheelSubsystem(HardwareMap hardwareMap) {
        // Initialize hardware components
        shoot = hardwareMap.get(DcMotor.class, "shoot");
        angles = hardwareMap.get(Servo.class, "angles");

        // Configure motor
        shoot.setDirection(DcMotorSimple.Direction.REVERSE);
        shoot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Ensure motor is using encoders for accurate position tracking
        shoot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Reset the timer and position data
        timer.reset();
        prevPos = shoot.getCurrentPosition();
    }

    /**
     * Sets the desired target RPM and the physical angle for the flywheel.
     */
    public void setTargetRPM(double rpm, double anglePosition) {
        this.TARGET_RPM = rpm;
        this.angles.setPosition(anglePosition);
        this.shooterDeriv = 0; // Reset integrator when target changes
        this.timer.reset(); // Reset timer to start new velocity tracking cycle
    }

    /**
     * Turns off the flywheel motor.
     */
    public void stop() {
        shoot.setPower(0);
        this.TARGET_RPM = 0;
        this.shooterDeriv = 0;
    }

    /**
     * The core PID update loop. Call this frequently (e.g., inside a while loop or OpMode loop).
     */
    public void update() {
        // 1. Calculate Velocity (in Ticks/Second)
        double currentPos = shoot.getCurrentPosition();
        double deltaTime = timer.seconds();

        if (deltaTime > 0.0) {
            // Your original velocity logic: velo = (shoot.getCurrentPosition()-prevpos)/(getRuntime() - prevtime1);
            currentVelo = (currentPos - prevPos) / deltaTime;
        } else {
            currentVelo = 0;
        }

        prevPos = currentPos;
        timer.reset();

        // 2. PID Calculation (only if a target is set)
        if (TARGET_RPM > 0) {
            double shooterErr = TARGET_RPM - currentVelo;

            shooterDeriv = shooterErr;

            // Calculate Power: P term + D term
            double shooterPower = (SKP * shooterErr) + (SKD * shooterDeriv);

            // 3. Set Motor Power
            shoot.setPower(clamp(shooterPower, 0, 1));
        }
    }

    /**
     * Checks if the flywheel velocity is stable enough to shoot.
     */
    public boolean isReadyToShoot() {
        if (TARGET_RPM == 0) return false;

        double tolerance = VELOCITY_TOLERANCE_PERCENT * TARGET_RPM;

        // Check if current velocity is within the defined tolerance range of the target
        return Math.abs(TARGET_RPM - currentVelo) < tolerance;
    }

    /**
     * Returns the last calculated velocity (in Ticks/Second).
     */
    public double getCurrentRPM() {
        return currentVelo;
    }

    public double clamp(double value, double min, double max) {
        return Math.min(Math.max(value, min), max);
    }
}