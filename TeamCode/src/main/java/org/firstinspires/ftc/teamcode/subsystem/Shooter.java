package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.common.InterpolatingMap;

import org.firstinspires.ftc.teamcode.common.RobotConstants;

import java.util.TreeMap;

public class Shooter {

    private final DcMotorEx fly;      // <-- DcMotorEx so we can read velocity reliably
    private final Servo kick;
    private final CRServo intakeServo;
    private final DcMotor intake;

    // RPM tracking
    private final ElapsedTime rpmTimer = new ElapsedTime();
    private double rpmFiltered = 0.0;
    private double targetRpm = 0.0;

    // Tuning
    private static final double RPM_ALPHA = 0.25; // 0..1 (higher = less smoothing)
    private static final double AT_SPEED_TOL_RPM = 50; // how close is "good enough"

    private final InterpolatingMap FlywheelMap = new InterpolatingMap();
    PIDFCoefficients pid = new PIDFCoefficients(10, 3, 0, 12);

    // TeleOp helper state: fire only once while button is held
    private boolean distanceShotFiredThisHold = false;




    public Shooter(HardwareMap hw) {
        fly  = (DcMotorEx) hw.dcMotor.get(RobotConstants.M_FLY);
        kick = hw.servo.get(RobotConstants.S_KICK);
        intake = hw.dcMotor.get(RobotConstants.M_INTAKE);
        intakeServo = hw.crservo.get(RobotConstants.S_INTAKE);


        fly.setDirection(DcMotorSimple.Direction.REVERSE);
        fly.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Encoder needed for velocity
        fly.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fly.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fly.setVelocityPIDFCoefficients(10, 3, 0, 12);

        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        fly.setPower(0);
        intake.setPower(0);
        kick.setPosition(RobotConstants.KICK_RETRACT);

        rpmTimer.reset();
        //         (Distance, power/rpm)
        FlywheelMap.put(0.0, 670.0);
        FlywheelMap.put(0.01, 670.0);   // close shot | 21in
        FlywheelMap.put(0.041, 770);//Peak of mid triangle | 77in
        FlywheelMap.put(0.88, 770); //TBH we just putting points in
        FlywheelMap.put(0.475, 940.0);  // far shot | 116in
    }

    /** Call this every loop (TeleOp/Auto) to keep rpmFiltered updated. */
    public void update() {
        double ticksPerSecond = fly.getVelocity(); // encoder ticks / second
        double rpmNow = (ticksPerSecond / RobotConstants.TICKS_PER_MOTOR_REV) * 60.0;

        // Exponential moving average filter
        rpmFiltered = (RPM_ALPHA * rpmNow) + ((1.0 - RPM_ALPHA) * rpmFiltered);
    }

    /** Measured (filtered) flywheel RPM. */
    public double getFlywheelRpm() {
        return rpmFiltered;
    }

    /** Last commanded target RPM (what you WANT). */
    public double getTargetRpm() {
        return targetRpm;
    }

    /** True when we're close enough to the target to confidently shoot. */
    public boolean atSpeed() {
        return Math.abs(getFlywheelRpm() - targetRpm) <= AT_SPEED_TOL_RPM;
    }

    // ----- Flywheel controls -----
    public boolean flywheelAtSpeed(){
        return Math.abs(getFlywheelRpm() - targetRpm) <= AT_SPEED_TOL_RPM;
    }


    /** Velocity control in encoder ticks/sec based on requested RPM. */
    public void setFlywheelRpm(double rpm) {
        targetRpm = Math.max(0, rpm);
        double ticksPerSecond = (targetRpm / 60.0) * RobotConstants.TICKS_PER_MOTOR_REV;

        // Requires RUN_USING_ENCODER
        fly.setVelocity(ticksPerSecond);
    }



    /** If you still want raw power control sometimes. */
    public void setFlywheelPower(double p) {
        if (p > 1) p = 1;
        if (p < -1) p = -1;
        targetRpm = 0; // "no rpm target" when using power
        fly.setPower(p);
    }

    public void stop() {
        targetRpm = 0;
        fly.setPower(0);
    }

    public void intakeFW() {
        // reverse spinning; RPM target doesn’t really apply here
        targetRpm = 0;
        fly.setPower(RobotConstants.FLY_SPEED_REVERSE);
    }

    // Presets (edit these to match your robot)
    public void closeShoot() { setFlywheelRpm(670); }
    public void farShoot()   { setFlywheelRpm(1000); }

    public void customShoot(double rpm, LinearOpMode op){
        setFlywheelRpm(rpm);
        if (flywheelAtSpeed()){
            feedOne(op);
            intake();
            op.sleep(RobotConstants.KICK_TIME_MS);
        }
    }

    public void intake()  {
        intake.setPower(1);
        intakeServo.setPower(1);
    }
    public void intakeReverse()  {
        intake.setPower(-1);
        intakeServo.setPower(-1);
    }
    public void stopIntake() { intake.setPower(0); intakeServo.setPower(0);}

    // ----- Kicker controls -----
    public void setKicker(boolean extended) {
        kick.setPosition(extended ? RobotConstants.KICK_EXTEND : RobotConstants.KICK_RETRACT);
    }

    public void flick(LinearOpMode op) {
        setKicker(true);
        op.sleep(RobotConstants.KICK_TIME_MS);
        setKicker(false);
    }

    public void feedOne(LinearOpMode op) {
        flick(op);
    }

    private void spinUpAndWait(Shooter shooter, double targetRpm, double timeoutSec) {
        shooter.setFlywheelRpm(targetRpm);
    }

    public double shootByDistance(double distance, LinearOpMode op){
        customShoot(FlywheelMap.get(distance), op);
        return FlywheelMap.get(distance);
    }

    /**
     * TeleOp helper for "hold to spin up" behavior.
     * While held, this continuously updates flywheel target based on distance and fires
     * exactly once when at speed. Releasing the button re-arms the next shot.
     */
    public double shootByDistanceHoldOnce(boolean triggerHeld, double distance, LinearOpMode op) {
        double mappedRpm = FlywheelMap.get(distance);

        if (!triggerHeld) {
            distanceShotFiredThisHold = false;
            setKicker(false);
            return mappedRpm;
        }

        setFlywheelRpm(mappedRpm);

        if (!distanceShotFiredThisHold && flywheelAtSpeed()) {
            feedOne(op);
            intake();
            distanceShotFiredThisHold = true;
        }

        return mappedRpm;
    }
}
