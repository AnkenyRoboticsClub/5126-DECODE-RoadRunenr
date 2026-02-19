package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystem.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystem.ImuUtil;
import org.firstinspires.ftc.teamcode.subsystem.VisionAlign;

@Autonomous(name = "AprilTag Rotate", group = "Auto")
public class AprilTagRotateAlignAuto extends LinearOpMode {

    @Override
    public void runOpMode() {
        DriveTrain drive = new DriveTrain(hardwareMap);
        ImuUtil imu = new ImuUtil(hardwareMap);
        VisionAlign vision = new VisionAlign(drive, imu);

        imu.resetYaw();
        vision.start(hardwareMap);

        telemetry.addLine("Ready: waits for any AprilTag, then rotates to face it");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) {
            vision.stop();
            drive.stopAll();
            return;
        }

        while (opModeIsActive()) {
            int tagId = vision.getAnyTagId();
            boolean hasTag = tagId >= 0;
            boolean aligned = false;

            if (hasTag) {
                aligned = vision.faceAnyTagStepRobotCentric();
            } else {
                // Requested behavior: stay still until any AprilTag appears.
                drive.stopAll();
            }

            telemetry.addData("Tag ID", tagId);
            telemetry.addData("Has Tag", hasTag);
            telemetry.addData("Aligned (within tol)", aligned);
            telemetry.update();

            sleep(20);
        }

        drive.stopAll();
        vision.stop();
    }
}
