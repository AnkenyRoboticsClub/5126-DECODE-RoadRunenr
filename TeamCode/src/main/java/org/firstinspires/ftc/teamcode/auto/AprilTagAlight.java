package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystem.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystem.ImuUtil;
import org.firstinspires.ftc.teamcode.subsystem.VisionAlign;

@Disabled
@Autonomous(name = "AprilTag Rotate Align", group = "Auto")
public class AprilTagAlight extends LinearOpMode {
    private static final double SEARCH_TURN_POWER = 0.20;
    private static final long ALIGN_HOLD_MS = 300;

    @Override
    public void runOpMode() {
        DriveTrain drive = new DriveTrain(hardwareMap);
        ImuUtil imu = new ImuUtil(hardwareMap);
        VisionAlign vision = new VisionAlign(drive, imu);

        imu.resetYaw();
        vision.start(hardwareMap);

        telemetry.addLine("Ready: tracking AprilTag rotation only");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) {
            vision.stop();
            drive.stopAll();
            return;
        }

        long alignedSinceMs = -1;

        while (opModeIsActive()) {
            int tagId = vision.getTagId();
            boolean hasTag = tagId >= 0;
            boolean aligned = false;

            if (hasTag) {
                aligned = vision.faceTagStepRobotCentric();
                if (aligned) {
                    if (alignedSinceMs < 0) {
                        alignedSinceMs = System.currentTimeMillis();
                    }
                } else {
                    alignedSinceMs = -1;
                }
            } else {
                // No tag visible yet: slowly rotate so we can acquire one.
                // drive.driveRobot(0, 0, SEARCH_TURN_POWER);
                // alignedSinceMs = -1;
            }

            boolean done = alignedSinceMs > 0 && (System.currentTimeMillis() - alignedSinceMs) >= ALIGN_HOLD_MS;

            telemetry.addData("Tag ID", tagId);
            telemetry.addData("Distance (m)", "%.2f", vision.getDistance());
            telemetry.addData("Has Tag", hasTag);
            telemetry.addData("Aligned", aligned);
            telemetry.addData("Done", done);
            telemetry.update();

            if (done) {
                break;
            }

            sleep(20);
        }

        drive.stopAll();
        vision.stop();
    }
}