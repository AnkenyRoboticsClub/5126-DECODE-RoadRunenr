package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystem.AutoDrive;
import org.firstinspires.ftc.teamcode.subsystem.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystem.ImuUtil;
import org.firstinspires.ftc.teamcode.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.subsystem.VisionAlign;

@Autonomous(name = "LL Shoot 1", group = "Auto")
public class LimeLightAuto extends LinearOpMode {

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
        drive.driveReverse();     // simple timed reverse
        sleep(900);    // tune this (700–1200 ms works)
        drive.stopAll();

        sleep(150); // settle

        // ---------------------------
        // 2. AIM TO CENTER TAG
        // ---------------------------
        vision.aimUntil(this);

        sleep(100);

        // ---------------------------
        // 3. BACK UP USING LL TARGET AREA
        // (robot moves until ta reaches Constants.LL_TARGET_AREA)
        // ---------------------------
        vision.aimAndApproachUntil(this);

        // ---------------------------
        // 4. SHOOT ONE
        // ---------------------------
        shooter.farShoot();
        sleep(800);            // wait for flywheel to stabilize

        shooter.feedOne(this);

        shooter.stop();
        drive.stopAll();

        telemetry.addLine("LL Auto Done!");
        telemetry.update();
        sleep(500);
    }
}
