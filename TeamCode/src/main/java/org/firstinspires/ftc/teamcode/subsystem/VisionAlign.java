package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.common.RobotConstants;

public class VisionAlign {

    // ---------------------------- Tuning knobs ----------------------------

    // Your camera is RIGHT SIDE UP. Leave this false.
    // If you ever mount the Limelight 180° (upside down), set true.
    private static final boolean CAMERA_UPSIDE_DOWN = false;

    // Noise deadband (deg). Under this, we do 0 turn to avoid wiggle.
    private static final double BEARING_DEADBAND_DEG = 1.5;

    // "Close enough" to stop (deg). If you already have this in RobotConstants, keep using it.
    // We'll use RobotConstants.LL_AIM_TOL_DEG as the final "done" check.

    // Simple smoothing for bearing to kill jitter (0..1). Higher = more responsive, lower = smoother.
    private static final double BEARING_ALPHA = 0.35;

    // Optional: cap how fast the turn command can change each loop (prevents snap)
    private static final double TURN_SLEW_PER_LOOP = 0.08;

    // ---------------------------- Members ----------------------------

    private final DriveTrain drive;
    private final ImuUtil imu; // not required for robot-centric, but keeping because you pass it in
    private Limelight3A limelight;

    private double filteredBearingDeg = 0.0;
    private double lastTurnCmd = 0.0;

    public VisionAlign(DriveTrain drive, ImuUtil imu) {
        this.drive = drive;
        this.imu = imu;
    }

    public void start(HardwareMap hw) {
        limelight = hw.get(Limelight3A.class, RobotConstants.LL_DEVICE_NAME);
        limelight.setPollRateHz(100);
        limelight.start();
    }

    public void stop() {
        if (limelight != null) limelight.stop();
    }

    public LLResult latest() {
        if (limelight == null) return null;
        return limelight.getLatestResult();
    }

    // ---------------------------- TeleOp step functions ----------------------------

    // Rotate ONLY until the robot is facing the GOAL tag (robot-space pose, not tx)
    public boolean faceTagStepRobotCentric() {
        return faceTagInternal(true);
    }

    // Rotate to face ANY visible tag
    public boolean faceAnyTagStepRobotCentric() {
        return faceTagInternal(false);
    }

    // Backwards compatible: your other code calls these
    public boolean aimStepRobotCentric() {
        // Keep this as a synonym for "face goal tag"
        return faceTagStepRobotCentric();
    }

    public boolean aimAndApproachStepRobotCentric() {
        LLResult r = latest();
        FiducialResult tag = findGoalTag(r);
        if (r == null || !r.isValid() || tag == null) {
            drive.stopAll();
            return false;
        }

        Pose3D tagPoseRobot = tag.getTargetPoseRobotSpace();
        if (tagPoseRobot == null) {
            drive.stopAll();
            return false;
        }

        // Bearing for turning
        double bearingDeg = computeBearingDeg(tagPoseRobot);
        bearingDeg = smoothBearing(bearingDeg);

        double turn = turnCmdStable(bearingDeg);

        // Forward based on target area (your existing logic)
        double fwd = forwardCmd(r.getTa());

        // Drive robot-centric: (strafe, forward, turn) -> your drive expects (x, y, rx) style
        drive.driveRobot(0, fwd, turn);

        return onTarget(bearingDeg, r.getTa());
    }

    public boolean faceTagUntil(LinearOpMode op) {
        ElapsedTime t = new ElapsedTime();
        while (op.opModeIsActive() && t.seconds() < RobotConstants.LL_ALIGN_TIMEOUT_S) {
            if (faceTagStepRobotCentric()) break;
            op.idle();
        }
        drive.stopAll();
        return true;
    }

    public boolean aimUntil(LinearOpMode op) {
        return faceTagUntil(op);
    }

    public boolean aimAndApproachUntil(LinearOpMode op) {
        ElapsedTime t = new ElapsedTime();
        while (op.opModeIsActive() && t.seconds() < RobotConstants.LL_APPROACH_TIMEOUT_S) {
            if (aimAndApproachStepRobotCentric()) break;
            op.idle();
        }
        drive.stopAll();
        return true;
    }

    // ---------------------------- Tag utilities ----------------------------

    public int getTagId() {
        LLResult r = latest();
        FiducialResult tag = findGoalTag(r);
        if (tag == null) return -1;
        return tag.getFiducialId();
    }

    public int getAnyTagId() {
        LLResult r = latest();
        FiducialResult tag = findAnyTag(r);
        if (tag == null) return -1;
        return tag.getFiducialId();
    }

    public String motifFromTag(int id) {
        switch (id) {
            case 21: return "GPP";
            case 22: return "PGP";
            case 23: return "PPG";
            default: return "UNKNOWN";
        }
    }

    public String scanMotif() {
        int id = getTagId();
        return motifFromTag(id);
    }

    private FiducialResult findAnyTag(LLResult r) {
        if (r == null || !r.isValid()) return null;
        java.util.List<FiducialResult> tags = r.getFiducialResults();
        if (tags == null || tags.isEmpty()) return null;
        return tags.get(0);
    }

    private FiducialResult findGoalTag(LLResult r) {
        if (r == null || !r.isValid()) return null;

        java.util.List<FiducialResult> tags = r.getFiducialResults();
        if (tags == null || tags.isEmpty()) return null;

        for (FiducialResult tag : tags) {
            int id = tag.getFiducialId();
            if (id == RobotConstants.LL_BLUE_GOAL_TAG_ID || id == RobotConstants.LL_RED_GOAL_TAG_ID) {
                return tag;
            }
        }
        return null;
    }

    // ---------------------------- Core logic ----------------------------

    private boolean faceTagInternal(boolean goalOnly) {
        LLResult r = latest();
        if (r == null || !r.isValid()) {
            drive.stopAll();
            return false;
        }

        FiducialResult tag = goalOnly ? findGoalTag(r) : findAnyTag(r);
        if (tag == null) {
            drive.stopAll();
            return false;
        }

        Pose3D tagPoseRobot = tag.getTargetPoseRobotSpace();
        if (tagPoseRobot == null) {
            drive.stopAll();
            return false;
        }

        double bearingDeg = computeBearingDeg(tagPoseRobot);
        bearingDeg = smoothBearing(bearingDeg);

        // Deadband so it stops hunting when basically centered
        if (Math.abs(bearingDeg) <= BEARING_DEADBAND_DEG) {
            drive.driveRobot(0, 0, 0);
            lastTurnCmd = 0.0;
            return true;
        }

        double turn = turnCmdStable(bearingDeg);
        drive.driveRobot(0, 0, turn);

        return Math.abs(bearingDeg) <= RobotConstants.LL_AIM_TOL_DEG;
    }

    private double computeBearingDeg(Pose3D tagPoseRobot) {
        Position p = tagPoseRobot.getPosition();
        double x = p.x; // forward
        double y = p.y; // sideways

        // If camera is upside down, y flips (and often tx/ty too).
        // You're right-side up, so this stays as-is.
        if (CAMERA_UPSIDE_DOWN) {
            y = -y;
        }

        // One consistent convention everywhere:
        // bearing = atan2(y, x)
        return Math.toDegrees(Math.atan2(y, x));
    }

    private double smoothBearing(double bearingDeg) {
        // low-pass filter
        filteredBearingDeg = filteredBearingDeg + BEARING_ALPHA * (bearingDeg - filteredBearingDeg);
        return filteredBearingDeg;
    }

    // Turn command with: proportional + min turn + clamp + slew-rate limit
    private double turnCmdStable(double bearingDeg) {
        // Final tolerance: if inside aim tolerance, stop.
        if (Math.abs(bearingDeg) <= RobotConstants.LL_AIM_TOL_DEG) {
            lastTurnCmd = 0.0;
            return 0.0;
        }

        double u = RobotConstants.LL_TURN_DIRECTION * RobotConstants.LL_K_TURN * bearingDeg;

        // Static friction bump ONLY when you're not super close, otherwise it causes overshoot.
        if (Math.abs(bearingDeg) > 3.0) {
            u += Math.signum(u) * RobotConstants.LL_MIN_TURN;
        }

        u = clamp(u, -RobotConstants.LL_MAX_TURN, RobotConstants.LL_MAX_TURN);

        // Slew limit so it can’t jump violently frame-to-frame
        double delta = clamp(u - lastTurnCmd, -TURN_SLEW_PER_LOOP, TURN_SLEW_PER_LOOP);
        lastTurnCmd += delta;

        return lastTurnCmd;
    }

    public double getDistance() {
        LLResult r = latest();
        if (r == null || !r.isValid()) return -1;

        FiducialResult tag = findGoalTag(r);
        if (tag == null) return -1;

        Pose3D tagPoseRobot = tag.getTargetPoseRobotSpace();
        if (tagPoseRobot == null) return -1;

        Position p = tagPoseRobot.getPosition();
        double x = p.x;
        double y = p.y;
        return Math.hypot(x, y);
    }

    // ---------------------------- Existing helpers ----------------------------

    private static double forwardCmd(double ta) {
        double err = RobotConstants.LL_TARGET_AREA - ta;
        if (Math.abs(err) <= RobotConstants.LL_APPROACH_TOL_TA) return 0;

        double u = RobotConstants.LL_K_FORWARD * err;
        u += Math.signum(u) * RobotConstants.LL_MIN_FORWARD;
        return clamp(u, -RobotConstants.LL_MAX_FORWARD, RobotConstants.LL_MAX_FORWARD);
    }

    private static boolean onTarget(double bearingDeg, double ta) {
        return Math.abs(bearingDeg) <= RobotConstants.LL_AIM_TOL_DEG &&
                ta >= RobotConstants.LL_TARGET_AREA - RobotConstants.LL_APPROACH_TOL_TA;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
