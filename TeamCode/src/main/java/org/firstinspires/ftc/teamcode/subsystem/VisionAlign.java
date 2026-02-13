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

    private final DriveTrain drive;
    private final ImuUtil imu;
    private Limelight3A limelight;

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

    // Rotate ONLY until the robot is facing the tag (using robot-space pose, not tx)
    public boolean faceTagStepRobotCentric() {
        LLResult r = latest();
        if (r == null || !r.isValid()) {
            drive.stopAll();
            return false;
        }

        java.util.List<FiducialResult> tags = r.getFiducialResults();
        if (tags == null || tags.isEmpty()) {
            drive.stopAll();
            return false;
        }

        FiducialResult tag = tags.get(0);
        Pose3D tagPoseRobot = tag.getTargetPoseRobotSpace();
        if (tagPoseRobot == null) {
            drive.stopAll();
            return false;
        }

        Position p = tagPoseRobot.getPosition();

        // Limelight FTC convention is typically:
        // x = forward, y = right (units usually meters)
        double x = p.x;
        double y = p.y;

        // Bearing to the tag in robot frame (radians -> degrees)
        double bearingDeg = Math.toDegrees(Math.atan2(y, x));
        //If turning weird, add (-) to the y

        double turn = turnCmd(bearingDeg);
        drive.driveRobot(0, 0, turn);

        return Math.abs(bearingDeg) <= RobotConstants.LL_AIM_TOL_DEG;
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


    // ---------------------------- TeleOp step functions ----------------------------

    public boolean aimStepRobotCentric() { //rotates the robot to center the tag horizontally
        LLResult r = latest();
        if (r == null || !r.isValid()) {
            drive.stopAll();
            return false;
        }

        double turn = turnCmd(r.getTx());
        drive.driveRobot(0, 0, turn);
        return Math.abs(r.getTx()) <= RobotConstants.LL_AIM_TOL_DEG;
    }

    public boolean aimAndApproachStepRobotCentric() { //Try and centers the robot with the Tag and approaches it
        LLResult r = latest();
        if (r == null || !r.isValid()) {
            drive.stopAll();
            return false;
        }

        double turn = turnCmd(r.getTx());
        double fwd  = forwardCmd(r.getTa());
        drive.driveRobot(0, fwd, turn);
        return onTarget(r.getTx(), r.getTa());
    }

    // ---------------------------- Auto blocking functions ----------------------------

    public boolean aimUntil(LinearOpMode op) {
        ElapsedTime t = new ElapsedTime();
        while (op.opModeIsActive() && t.seconds() < RobotConstants.LL_ALIGN_TIMEOUT_S) {
            if (aimStepRobotCentric()) break;
            op.idle();
        }
        drive.stopAll();
        return true;
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

    public int getTagId() {
        LLResult r = latest();
        if (r == null || !r.isValid()) return -1;

        // Get list of fiducial (AprilTag) detections
        java.util.List<FiducialResult> tags = r.getFiducialResults();

        if (tags == null || tags.isEmpty()) {
            return -1;  // No tags detected
        }

        // Return ID of the first (main) tag
        return tags.get(0).getFiducialId();
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
    // ---------------------------- Math Helpers ----------------------------

    public double getDistance() {
        LLResult r = latest();
        if (r == null || !r.isValid()) return -1;

        java.util.List<FiducialResult> tags = r.getFiducialResults();
        if (tags == null || tags.isEmpty()) return -1;

        FiducialResult tag = tags.get(0);

        // Tag position relative to the robot (robot-space)
        Pose3D tagPoseRobot = tag.getTargetPoseRobotSpace();  // exists in the FTC Limelight API :contentReference[oaicite:1]{index=1}
        if (tagPoseRobot == null) return -1;

        Position p = tagPoseRobot.getPosition(); // :contentReference[oaicite:2]{index=2}

        // NOTE: In FTC SDK, Position stores x/y/z in its DistanceUnit.
        // Commonly these are meters when coming from Limelight Pose3D.
        double x = p.x; // forward (meters)
        double y = p.y; // right   (meters)
        return Math.hypot(x, y); // ground distance (meters)
    }


    private static double turnCmd(double tx) {
        if (Math.abs(tx) <= RobotConstants.LL_AIM_TOL_DEG) return 0;

        double u = RobotConstants.LL_K_TURN * tx;
        u += Math.signum(u) * RobotConstants.LL_MIN_TURN;
        return clamp(u, -RobotConstants.LL_MAX_TURN, RobotConstants.LL_MAX_TURN);
    }

    private static double forwardCmd(double ta) {
        double err = RobotConstants.LL_TARGET_AREA - ta;
        if (Math.abs(err) <= RobotConstants.LL_APPROACH_TOL_TA) return 0;

        double u = RobotConstants.LL_K_FORWARD * err;
        u += Math.signum(u) * RobotConstants.LL_MIN_FORWARD;
        return clamp(u, -RobotConstants.LL_MAX_FORWARD, RobotConstants.LL_MAX_FORWARD);
    }

    private static boolean onTarget(double tx, double ta) {
        return Math.abs(tx) <= RobotConstants.LL_AIM_TOL_DEG &&
                ta >= RobotConstants.LL_TARGET_AREA - RobotConstants.LL_APPROACH_TOL_TA;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
