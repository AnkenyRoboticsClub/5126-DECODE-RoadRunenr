package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.common.RobotConstants;

public class AutoDrive {
    private final DcMotor fl, fr, bl, br;

    public AutoDrive(HardwareMap hw) {
        fl = hw.dcMotor.get(RobotConstants.M_FL);
        fr = hw.dcMotor.get(RobotConstants.M_FR);
        bl = hw.dcMotor.get(RobotConstants.M_BL);
        br = hw.dcMotor.get(RobotConstants.M_BR);

        // Directions (match your original)
        fl.setDirection(DcMotorSimple.Direction.FORWARD);
        bl.setDirection(DcMotorSimple.Direction.FORWARD);
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);

        for (DcMotor m : new DcMotor[]{fl, fr, bl, br}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }
    //=======================AUTO=====================
    // --- tune these at the top of DriveTrain or near your fields ---
    static final double TICKS_PER_REV = 28;   // set for your motor
    static final double WHEEL_DIAMETER_IN = 3.78;// set for your wheel
    static final double GEAR_RATIO = 1.0;        // output gear ratio to wheel
    
    private double inchesToTicks(double inches) {
        double circumference = Math.PI * WHEEL_DIAMETER_IN;
        double revs = (inches / circumference) * GEAR_RATIO;
        return revs * TICKS_PER_REV;
    }
    
    public void resetDriveEncoders() {
        for (DcMotor m : new DcMotor[]{fl, fr, bl, br}) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }
    
    public int averageAbsTicks() {
        return (Math.abs(fl.getCurrentPosition()) + Math.abs(fr.getCurrentPosition())
               + Math.abs(bl.getCurrentPosition()) + Math.abs(br.getCurrentPosition())) / 4;
    }
    
    public void stopAll() {
        fl.setPower(0); fr.setPower(0); bl.setPower(0); br.setPower(0);
    }
    
    public void aidenTurn(){
        fl.setPower(.6); fr.setPower(-.4); bl.setPower(-.4); br.setPower(.6);
    }

    /** Drive straight (robot-centric) for inches at given power using encoders. */
    public void driveStraightInches(LinearOpMode op, double inches, double maxPower) {
        resetDriveEncoders();

        // Convert inches → ticks using RobotConstants
        int targetTicks = (int) Math.round(inches * RobotConstants.TICKS_PER_INCH);
        double direction = Math.signum(inches);

        maxPower = Math.abs(maxPower) * direction;

        double minPower = 0.12;
        double accelDist = 0.25;
        double decelDist = 0.25;

        while (op.opModeIsActive()) {

            int current = averageAbsTicks();
            int absTarget = Math.abs(targetTicks);

            if (current >= absTarget) break;

            double progress = (double) current / absTarget;

            double commandedPower;

            if (progress < accelDist) {
                double scale = progress / accelDist;
                commandedPower = lerp(minPower, maxPower, scale);
            }
            else if (progress < 1.0 - decelDist) {
                commandedPower = maxPower;
            }
            else {
                double scale = (1.0 - progress) / decelDist;
                commandedPower = lerp(minPower, maxPower, scale);
            }

            fl.setPower(commandedPower);
            fr.setPower(commandedPower);
            bl.setPower(commandedPower);
            br.setPower(commandedPower);

            op.idle();
        }
        stopAll();
    }

    private double lerp(double a, double b, double t) {
        return a + (b - a) * t;
    }
    
    public void driveReverse(){
            fl.setPower(-.3);
            bl.setPower(-.3);
            fr.setPower(-.3);
            br.setPower(-.3);
    }
    public void driveForwardSim(){
        fl.setPower(.5);
        bl.setPower(.5);
        fr.setPower(.5);
        br.setPower(.5);
    }
    
    /** Turn in place to an absolute heading (deg, -180..180) using IMU (simple P). */
    public void turnToHeadingDegrees(LinearOpMode op, ImuUtil imu, double targetDeg, double maxPower, double kP) {
        while (op.opModeIsActive()) {
            double currentDeg = Math.toDegrees(imu.getHeadingRad());
            double error = angleWrapDeg(targetDeg - currentDeg);
            if (Math.abs(error) < 1.5) break;
    
            double turn = kP * error;
            if (turn >  maxPower) turn =  maxPower;
            if (turn < -maxPower) turn = -maxPower;
    
            fl.setPower( -turn); bl.setPower( -turn);
            fr.setPower(turn); br.setPower(turn);
            op.idle();
        }
        stopAll();
    }
    
    private double angleWrapDeg(double a) {
        while (a > 180)  a -= 360;
        while (a <= -180) a += 360;
        return a;
    }

    /** Drive straight (robot-centric) for inches at given power using encoders + IMU heading hold.
     *  inches: +forward, -backward
     *  maxPower: positive number (0..1)
     */
    public void driveStraightInchesHoldHeading(
            LinearOpMode op,
            ImuUtil imu,
            double inches,
            double maxPower,
            double kP,
            double maxTurn
    ) {
        resetDriveEncoders();

        // Capture the heading you want to hold RIGHT NOW
        double targetDeg = Math.toDegrees(imu.getHeadingRad());

        int targetTicks = (int) Math.round(Math.abs(inches) * RobotConstants.TICKS_PER_INCH);
        double direction = Math.signum(inches); // +1 forward, -1 backward

        maxPower = Math.abs(maxPower); // we apply direction later

        double minPower = 0.20;
        double accelDist = 0.25;
        double decelDist = 0.25;

        while (op.opModeIsActive()) {
            int current = averageAbsTicks();
            if (current >= targetTicks) break;

            double progress = (double) current / (double) targetTicks;

            double commandedMag;
            if (progress < accelDist) {
                double scale = progress / accelDist;
                commandedMag = lerp(minPower, maxPower, scale);
            } else if (progress < 1.0 - decelDist) {
                commandedMag = maxPower;
            } else {
                double scale = (1.0 - progress) / decelDist;
                commandedMag = lerp(minPower, maxPower, scale);
            }

            // Base forward/back power
            double y = commandedMag * direction;


            // IMU heading correction
            double currentDeg = Math.toDegrees(imu.getHeadingRad());
            double error = angleWrapDeg(currentDeg - targetDeg);
            double rawTurn = kP * error;
            double turnLimit = Math.min(maxTurn, Math.abs(y) * 0.6);
            double turn = clip(rawTurn, -turnLimit, turnLimit);


            // Mecanum robot-centric, x = 0, apply turn
            // (This is the standard mix for forward/back + rotate)
            double flp = y + turn;
            double frp = y - turn;
            double blp = y + turn;
            double brp = y - turn;

            // Normalize so nothing exceeds 1.0
            double max = Math.max(Math.max(Math.abs(flp), Math.abs(frp)),
                    Math.max(Math.abs(blp), Math.abs(brp)));
            if (max > 1.0) {
                flp /= max; frp /= max; blp /= max; brp /= max;
            }

            fl.setPower(flp);
            fr.setPower(frp);
            bl.setPower(blp);
            br.setPower(brp);

            op.telemetry.addData("targetDeg", targetDeg);
            op.telemetry.addData("currentDeg", currentDeg);
            op.telemetry.addData("error", error);
            op.telemetry.addData("turn", turn);
            op.telemetry.update();

            op.idle();
        }

        stopAll();
    }

    private double clip(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }


}
