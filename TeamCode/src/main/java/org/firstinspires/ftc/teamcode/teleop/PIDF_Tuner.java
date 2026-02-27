package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.Shooter;

/**
 * ============================================================
 *  FLYWHEEL PIDF TUNER  –  Practice-field use only
 * ============================================================
 *
 *  CONTROLLER MAP (Gamepad 1)
 *  ─────────────────────────────────────────────────────────
 *  Y              → Spin flywheel at target RPM
 *  A              → Stop flywheel
 *  B              → Fire kicker (manual flick)
 *  X              → Toggle intake on/off
 *
 *  D-Pad UP/DOWN  → Increase / decrease TARGET RPM  (+/- 10)
 *  Left  Bumper   → Hold to adjust P  │  Right Bumper → Hold to adjust F
 *  (while bumper held, D-Pad UP/DOWN changes that coefficient by step)
 *
 *  Left  Trigger  → Hold to adjust I  │  Right Trigger → Hold to adjust D
 *  (same D-Pad UP/DOWN pattern)
 *
 *  START          → Apply current PIDF to motor (re-sends coefficients)
 *
 *  ─────────────────────────────────────────────────────────
 *  TUNING GUIDE (printed in telemetry every loop)
 *
 *  Recommended order:  F → P → D → I (last)
 *
 *  F  –  Feedforward.  Start here.  Raise F until the flywheel reaches
 *         ~80-90 % of target without PID fighting it.
 *         Too low  → PID overworks, oscillation.
 *         Too high → overshoot on start.
 *         Typical range: 10 – 15 for a 28:1 gearbox @ 12V.
 *
 *  P  –  Proportional.  Closes the remaining gap.
 *         Too high → oscillation / buzzing sound on the motor.
 *         Too low  → slow to settle.
 *         Tune with I=0, D=0 first.
 *
 *  D  –  Derivative.  Add small D only if there is overshooting/ringing.
 *         Start at 0, nudge up in 0.5 steps.  Rarely needs to be > 2.
 *
 *  I  –  Integral.  Add last, in tiny steps (0.1 – 0.5).
 *         Helps eliminate steady-state error under load.
 *         Too high → wind-up during spool-up (big oscillation).
 *
 *  Good "at-speed" indicator: watch ERROR RPM.  When it stays within
 *  ± 50 RPM consistently, you're ready to shoot.
 * ============================================================
 */
@TeleOp(name = "Flywheel PIDF Tuner", group = "Tuning")
public class PIDF_Tuner extends LinearOpMode {

    // ── Adjustment steps ───────────────────────────────────
    private static final double P_STEP   = 0.5;
    private static final double I_STEP   = 0.1;
    private static final double D_STEP   = 0.25;
    private static final double F_STEP   = 0.5;
    private static final double RPM_STEP = 10.0;

    // ── Debounce timing (ms) ───────────────────────────────
    private static final long DPAD_DEBOUNCE_MS = 175;

    private Shooter shooter;

    // Runtime state
    private double targetRpm    = 600.0;
    private boolean intakeOn    = false;
    private boolean flywheelOn  = false;

    // Debounce timestamps
    private long lastDpadTime   = 0;
    private long lastStartTime  = 0;
    private long lastXTime      = 0;
    private long lastBTime      = 0;

    @Override
    public void runOpMode() {

        shooter = new Shooter(hardwareMap);

        telemetry.addLine("FLYWHEEL PIDF TUNER – ready.  Press START to begin.");
        telemetry.addLine("See comments in FlywheelPIDFTuner.java for full control map.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            shooter.update();   // keep RPM filter fresh

            long now = System.currentTimeMillis();

            // ── Determine which coefficient is selected ────
            boolean adjP = gamepad1.left_bumper;
            boolean adjF = gamepad1.right_bumper;
            boolean adjI = gamepad1.left_trigger  > 0.5;
            boolean adjD = gamepad1.right_trigger > 0.5;
            // If nothing held, D-Pad changes RPM target
            boolean adjRpm = !(adjP || adjF || adjI || adjD);

            // ── D-Pad UP ───────────────────────────────────
            if (gamepad1.dpad_up && (now - lastDpadTime) > DPAD_DEBOUNCE_MS) {
                lastDpadTime = now;
                if      (adjP)   shooter.P   += P_STEP;
                else if (adjI)   shooter.I   += I_STEP;
                else if (adjD)   shooter.D   += D_STEP;
                else if (adjF)   shooter.F   += F_STEP;
                else             targetRpm   += RPM_STEP;
            }

            // ── D-Pad DOWN ─────────────────────────────────
            if (gamepad1.dpad_down && (now - lastDpadTime) > DPAD_DEBOUNCE_MS) {
                lastDpadTime = now;
                if      (adjP)   shooter.P   = Math.max(0, shooter.P - P_STEP);
                else if (adjI)   shooter.I   = Math.max(0, shooter.I - I_STEP);
                else if (adjD)   shooter.D   = Math.max(0, shooter.D - D_STEP);
                else if (adjF)   shooter.F   = Math.max(0, shooter.F - F_STEP);
                else             targetRpm   = Math.max(0, targetRpm  - RPM_STEP);
            }

            // ── START → apply PIDF to motor ───────────────
            if (gamepad1.start && (now - lastStartTime) > 400) {
                lastStartTime = now;
                shooter.applyPIDF();
                // Re-command current RPM so the new gains take effect immediately
                if (flywheelOn) shooter.setFlywheelRpm(targetRpm);
            }

            // ── Y → Spin up ────────────────────────────────
            if (gamepad1.y) {
                flywheelOn = true;
                shooter.setFlywheelRpm(targetRpm);
            }

            // ── A → Stop flywheel ──────────────────────────
            if (gamepad1.a) {
                flywheelOn = false;
                shooter.stop();
            }

            // ── B → Manual kicker flick ───────────────────
            if (gamepad1.b && (now - lastBTime) > 600) {
                lastBTime = now;
                shooter.feedOne(this);
            }

            // ── X → Toggle intake ─────────────────────────
            if (gamepad1.x && (now - lastXTime) > 400) {
                lastXTime = now;
                intakeOn = !intakeOn;
                if (intakeOn) shooter.intake();
                else          shooter.stopIntake();
            }

            // ── Telemetry ──────────────────────────────────
            double actualRpm  = shooter.getFlywheelRpm();
            double errorRpm   = actualRpm - targetRpm;
            boolean atSpeed   = shooter.flywheelAtSpeed();

            // Which param is actively being adjusted?
            String activeParam = adjP ? "P" : adjF ? "F" : adjI ? "I" : adjD ? "D" : "RPM";

            telemetry.addLine("════════  FLYWHEEL PIDF TUNER  ════════");
            telemetry.addLine();

            telemetry.addLine("── PIDF Coefficients ──────────────────");
            telemetry.addData("  P", "%.2f  %s", shooter.P, adjP ? "◄ EDITING" : "");
            telemetry.addData("  I", "%.2f  %s", shooter.I, adjI ? "◄ EDITING" : "");
            telemetry.addData("  D", "%.2f  %s", shooter.D, adjD ? "◄ EDITING" : "");
            telemetry.addData("  F", "%.2f  %s", shooter.F, adjF ? "◄ EDITING" : "");
            telemetry.addLine();

            telemetry.addLine("── RPM Status ─────────────────────────");
            telemetry.addData("  Target RPM ",  "%.0f  %s", targetRpm,  adjRpm ? "◄ EDITING" : "");
            telemetry.addData("  Actual RPM ",  "%.0f", actualRpm);
            telemetry.addData("  Error  RPM ",  "%+.0f", errorRpm);
            telemetry.addData("  At Speed?  ",  atSpeed ? "✓ YES" : "✗ NO");
            telemetry.addLine();

            telemetry.addLine("── State ───────────────────────────────");
            telemetry.addData("  Flywheel", flywheelOn ? "SPINNING" : "STOPPED");
            telemetry.addData("  Intake  ", intakeOn   ? "ON"       : "OFF");
            telemetry.addData("  Editing ", activeParam);
            telemetry.addLine();

            telemetry.addLine("── Controls ────────────────────────────");
            telemetry.addLine("  Y=Spin  A=Stop  B=Flick  X=Intake");
            telemetry.addLine("  LB=P  RB=F  LT=I  RT=D");
            telemetry.addLine("  D-UP/DOWN = adjust selected param");
            telemetry.addLine("  START = apply PIDF to motor");
            telemetry.addLine();

            telemetry.addLine("── Tuning Guide ────────────────────────");
            telemetry.addLine("  Order: F → P → D → I (tiny, last)");
            telemetry.addLine("  Oscillating? Lower P first, then add D");
            telemetry.addLine("  Slow settle? Raise P or F");
            telemetry.addLine("  Steady error under load? Small I (0.1-0.5)");
            telemetry.addLine("  ALWAYS press START after changing PIDF!");

            telemetry.update();

            idle();
        }

        // Clean up
        shooter.stop();
        shooter.stopIntake();
    }
}