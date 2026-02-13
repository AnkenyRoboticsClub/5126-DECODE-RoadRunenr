package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystem.AutoDrive;
import org.firstinspires.ftc.teamcode.subsystem.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystem.ImuUtil;
import org.firstinspires.ftc.teamcode.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.subsystem.VisionAlign;

@Autonomous(name = "CloseAuto3B", group = "Auto")
public class Ball3Close extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        AutoDrive drive = new AutoDrive(hardwareMap);
        ImuUtil imu = new ImuUtil(hardwareMap);
        Shooter shooter = new Shooter(hardwareMap);
        DriveTrain teleDrive = new DriveTrain(hardwareMap); // used for LL movement
        VisionAlign vision = new VisionAlign(teleDrive, imu);

        imu.resetYaw();
        vision.start(hardwareMap);

        telemetry.addLine("LL Auto Ready");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // ---------------------------
        // 1. BLIND BACKUP (so LL sees tag)
        // ---------------------------
        //drive.driveReverse();
        //sleep(1500);    // tune this (700–1200 ms works)
        //drive.stopAll();
        // Backup while holding heading
        drive.driveStraightInchesHoldHeading(this, imu, -6, 0.10, 0.05, 0.10);
        sleep(550);
        drive.turnToHeadingDegrees(this, imu, 0, 0.15, 1);
        sleep(1700);
        // ---------------------------
        // 4. SHOOT ONE
        // ---------------------------
        telemetry.addData("Distance", vision.getDistance());
        //shooter.shootByDistance(vision.getDistance(), this);
        shooter.customShoot(630, this);
        sleep(2000);            // wait for flywheel to stabilize
        telemetry.addData("RPM:", shooter.getFlywheelRpm());

        shooter.feedOne(this);

        // ---------------------------
        // 5. INTAKE 2nd ball into fly wheel
        // ---------------------------
        sleep(500);
        shooter.intake();
        sleep(3000);

        shooter.feedOne(this);

        // ---------------------------
        // 5. Move off line
        // ---------------------------


        shooter.stop();
        drive.stopAll();

        telemetry.addLine("LL Auto Done!");
        telemetry.update();
        sleep(500);
    }
}
