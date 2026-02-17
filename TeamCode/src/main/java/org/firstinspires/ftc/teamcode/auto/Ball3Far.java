package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystem.AutoDrive;
import org.firstinspires.ftc.teamcode.subsystem.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystem.ImuUtil;
import org.firstinspires.ftc.teamcode.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.subsystem.VisionAlign;

@Autonomous(name = "FarAuto3B", group = "Auto")
public class Ball3Far extends LinearOpMode {

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
        // 1. Aim at tag
        // ---------------------------
        vision.faceTagUntil(this);
        sleep(250);

        // ---------------------------
        // 2. Spin to far-shot RPM
        // ---------------------------
        shooter.farShoot();
        shooter.waitForAtSpeed(this, 2500);

        // ---------------------------
        // 3. Kick first ball
        // ---------------------------
        shooter.feedOne(this);

        // ---------------------------
        // 4. Wait for RPM recovery, then intake next 2
        // ---------------------------
        shooter.waitForAtSpeed(this, 1500);
        shooter.intake();
        sleep(2200);
        shooter.stopIntake();

        // ---------------------------
        // 5. Move off line
        // ---------------------------
        drive.driveForwardSim();
        sleep(500);

        shooter.stop();
        drive.stopAll();

        telemetry.addLine("LL Auto Done!");
        telemetry.update();
        sleep(500);
    }
}
