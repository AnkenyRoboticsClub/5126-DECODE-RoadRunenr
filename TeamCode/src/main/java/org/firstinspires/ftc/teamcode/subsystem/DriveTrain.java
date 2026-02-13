package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.common.RobotConstants;

/*
 * If im being honest, I have 0 clue how this works, you would think
 * after 5 years of FTC I would understand, but ive just been using
 * the same felid centric mechanum code that Gabe made forever ago
 * he claimes it was copy paste from game maual 0, but to me its
 * wichcraft, so modify only if you know what your doing, OR just 
 * re-write everything from scratch, who cares, not the robot
 */
public class DriveTrain {
    public final DcMotorEx fl, fr, bl, br;
    static final double TICKS_PER_REV = 537.6;

    public DriveTrain(HardwareMap hw) {
        fl = hw.get(DcMotorEx.class, RobotConstants.M_FL);
        fr = hw.get(DcMotorEx.class, RobotConstants.M_FR);
        bl = hw.get(DcMotorEx.class, RobotConstants.M_BL);
        br = hw.get(DcMotorEx.class, RobotConstants.M_BR);
        /*
        vr = hw.dcMotor.get(Constants.M_VR);
        vl = hw.dcMotor.get(Constants.M_VL);
        */

        // Directions (match your original)
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.FORWARD);
        br.setDirection(DcMotorSimple.Direction.FORWARD);

        /*
        vr.setDirection(DcMotorSimple.Direction.FORWARD);
        vl.setDirection(DcMotorSimple.Direction.FORWARD);
         */
        // Optionally set ZeroPowerBehavior, run modes, etc.
        for (DcMotor m : new DcMotor[]{fl, fr, bl, br}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    /** Field-centric drive. Inputs x,y,rx are gamepad values; headingRad from IMU. */
    public void driveFieldCentric(double x, double y, double rx, double headingRad,
                           boolean slow, boolean fast, boolean rwd) {
        // Rotate the input vector by -heading (field-oriented)
        double rotX = x * Math.cos(-headingRad) - y * Math.sin(-headingRad);
        double rotY = x * Math.sin(-headingRad) + y * Math.cos(-headingRad);

        // Optional: scale turn speed like your original using trigger/bumper in OpMode
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1.0);
        double flP = (rotY + rotX + rx) / denominator;
        double blP = (rotY - rotX + rx) / denominator;
        double frP = (rotY - rotX - rx) / denominator;
        double brP = (rotY + rotX - rx) / denominator;

        double scale = RobotConstants.MOVING_SPEED;       // default
        if (fast) scale = RobotConstants.MOVING_SPEED_FAST;
        if (slow) scale = RobotConstants.MOVING_SPEED_SLOW;

        if (rwd){
            bl.setPower(blP * scale);
            br.setPower(brP * scale);
        }
        else{
            fl.setPower(flP * scale);
            bl.setPower(blP * scale);
            fr.setPower(frP * scale);
            br.setPower(brP * scale);
        }
    }

    public void nudgeLeft(LinearOpMode op)  { nudge(-RobotConstants.SCOOTCH_POWER,  RobotConstants.SCOOTCH_POWER,  RobotConstants.SCOOTCH_POWER, -RobotConstants.SCOOTCH_POWER, op); }
    public void nudgeRight(LinearOpMode op) { nudge( RobotConstants.SCOOTCH_POWER, -RobotConstants.SCOOTCH_POWER, -RobotConstants.SCOOTCH_POWER,  RobotConstants.SCOOTCH_POWER, op); }
    public void nudgeForward(LinearOpMode op){ nudge( RobotConstants.SCOOTCH_POWER,  RobotConstants.SCOOTCH_POWER,  RobotConstants.SCOOTCH_POWER,  RobotConstants.SCOOTCH_POWER, op); }
    public void nudgeBack(LinearOpMode op)  { nudge(-RobotConstants.SCOOTCH_POWER, -RobotConstants.SCOOTCH_POWER, -RobotConstants.SCOOTCH_POWER, -RobotConstants.SCOOTCH_POWER, op); }

    private void nudge(double flP, double frP, double blP, double brP, LinearOpMode op) {
        fl.setPower(flP); fr.setPower(frP); bl.setPower(blP); br.setPower(brP);
        op.sleep(RobotConstants.SCOOTCH_DURATION_MS);
    }

    // In DriveTrain.java
    // Robot-centric arcade for mecanum; same math you use in field-centric but without heading rotation.
    public void driveRobot(double x, double y, double rx) {
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
        double flP = (y + x + rx) / denominator;
        double blP = (y - x + rx) / denominator;
        double frP = (y - x - rx) / denominator;
        double brP = (y + x - rx) / denominator;

        fl.setPower(flP);
        bl.setPower(blP);
        fr.setPower(frP);
        br.setPower(brP);
    }
    
    public double getRPM(DcMotorEx motor) {
        double tps = motor.getVelocity();      // ticks/second
        if (Double.isNaN(tps)) tps = 0.0;      // safety
        return (tps * 60.0) / TICKS_PER_REV;   // convert to RPM
    }
    
    public void testDrive() {
        fl.setPower(.2);
        bl.setPower(.2);
        fr.setPower(.2);
        br.setPower(.2);
    }
    
    public void stopAll() {
        fl.setPower(0); fr.setPower(0); bl.setPower(0); br.setPower(0);
    }
    
    public void assistRight(){
        fl.setPower(RobotConstants.TURN_SPEED); fr.setPower(-RobotConstants.TURN_SPEED); bl.setPower(-RobotConstants.TURN_SPEED); br.setPower(RobotConstants.TURN_SPEED);
    }
    public void assistLeft(){
        fl.setPower(-RobotConstants.TURN_SPEED); fr.setPower(RobotConstants.TURN_SPEED); bl.setPower(RobotConstants.TURN_SPEED); br.setPower(-RobotConstants.TURN_SPEED);
    }
    
    /*
    void liftRobot(){
        vr.setPower(.5);
        vl.setPower(.5);
    }
    */
    
}
