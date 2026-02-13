package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.lynx.LynxModule;
import org.firstinspires.ftc.teamcode.subsystem.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystem.ImuUtil;
import org.firstinspires.ftc.teamcode.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.subsystem.VisionAlign;


@TeleOp(name="GREEN BOT", group="Linear OpMode")
public class GREENBOT extends LinearOpMode {
    private DriveTrain drive;
    private ImuUtil imu;
    private Shooter shooter;
    private VisionAlign vision;

    @Override
    public void runOpMode() {
        drive   = new DriveTrain(hardwareMap);
        imu     = new ImuUtil(hardwareMap);
        shooter = new Shooter(hardwareMap);
        vision  = new VisionAlign(drive, imu);
        vision.start(hardwareMap);

        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }


        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y  = -gamepad1.left_stick_y;
            double x  =  gamepad1.left_stick_x;
            double rx =  gamepad1.right_stick_x;

            if (gamepad1.options) imu.resetYaw();
            double heading = imu.getHeadingRad();

            boolean fast = gamepad1.right_trigger > 0.1;
            boolean slow = gamepad1.left_trigger  > 0.1;

            // Driver 1 aim-assist is non-blocking while A is held
            if (gamepad1.a) {
                vision.faceTagStepRobotCentric();
            } else {
                // Drive
                drive.driveFieldCentric(x, y, rx, heading, slow, fast, false);

                // Driver 1 manual assists
                if (gamepad1.left_bumper)  drive.assistLeft();
                if (gamepad1.right_bumper) drive.assistRight();
            }

            // Shooter
            shooter.update();

            //Driver 2 ====================
            if (gamepad2.right_trigger > 0.1) shooter.farShoot();
            else if (gamepad2.left_trigger > 0.1) shooter.closeShoot();
            else                              shooter.stop();

            if (gamepad2.right_bumper) shooter.intakeFW();// For temp human player feeding
            if (gamepad2.x) shooter.intakeReverse();

            if (gamepad2.left_bumper) shooter.intake();
            else                      shooter.stopIntake();

            if (gamepad2.a) shooter.feedOne(this); // extend + retract
            if (gamepad2.b) shooter.shootByDistance(vision.getDistance(), this);

            shooter.shootByDistanceHoldOnce(gamepad2.y, vision.getDistance(), this);
            //=============================
            /*
            if (gamepad1.dpad_right) drive.nudgeRight();
            if (gamepad1.dpad_left)  drive.nudgeLeft();
            if (gamepad1.dpad_up)    drive.nudgeForward();
            if (gamepad1.dpad_down)  drive.nudgeBack();
            */

            telemetry.addData("Flywheel", gamepad2.right_trigger > 0.1 ? "ON" : "OFF");
            telemetry.addData("Flywheel RPM", shooter.getFlywheelRpm());
            telemetry.addLine("--- DRIVE RPM ---");
            telemetry.addData("FL", "%.1f", drive.getRPM(drive.fl));
            telemetry.addData("FR", "%.1f", drive.getRPM(drive.fr));
            telemetry.addData("BL", "%.1f", drive.getRPM(drive.bl));
            telemetry.addData("BR", "%.1f", drive.getRPM(drive.br));
            double avgAbs = (Math.abs(drive.getRPM(drive.fl)) + Math.abs(drive.getRPM(drive.fr)) +
                             Math.abs(drive.getRPM(drive.bl)) + Math.abs(drive.getRPM(drive.br))) / 4.0;
            telemetry.addData("Avg (abs)", "%.1f", avgAbs);
            telemetry.addData("Aim Assist", gamepad1.a ? "ON (hold A)" : "OFF");
            telemetry.update();
        }

        vision.stop();
    }
}
