package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystem.AutoDrive;
import org.firstinspires.ftc.teamcode.subsystem.ImuUtil;
import org.firstinspires.ftc.teamcode.subsystem.Shooter;

@Autonomous(name = "CloseAuto3B", group = "Auto")
public class Ball3Close extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        AutoDrive drive = new AutoDrive(hardwareMap);
        ImuUtil imu = new ImuUtil(hardwareMap);
        Shooter shooter = new Shooter(hardwareMap);

        imu.resetYaw();
        telemetry.addLine("LL Auto Ready");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // ---------------------------
        // 1. Blind backup while holding heading
        // ---------------------------
        drive.driveStraightInchesHoldHeading(this, imu, 10, 0.10, 0.05, 0.10);
        sleep(750);

        // ---------------------------
        // 2. Spin up and kick first ball
        // ---------------------------
        shooter.setFlywheelRpm(630);
        shooter.waitForAtSpeed(this, 2200);
        shooter.feedOne(this);

        // ---------------------------
        // 3. Wait for recovery then intake next 2
        // ---------------------------
        shooter.waitForAtSpeed(this, 1200);
        shooter.intake();
        sleep(2500);
        shooter.stopIntake();

        shooter.stop();
        drive.stopAll();

        telemetry.addLine("LL Auto Done!");
        telemetry.update();
        sleep(500);
    }
}
