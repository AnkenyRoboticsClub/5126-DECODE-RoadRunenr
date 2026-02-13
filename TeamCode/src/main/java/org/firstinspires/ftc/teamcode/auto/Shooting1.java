package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystem.AutoDrive;
import org.firstinspires.ftc.teamcode.subsystem.ImuUtil;
import org.firstinspires.ftc.teamcode.subsystem.Shooter;

@Autonomous(name = "Shoot 1")
public class Shooting1 extends LinearOpMode {
    @Override
    public void runOpMode() {
        AutoDrive drive = new AutoDrive(hardwareMap); // same as TeleOp
        Shooter shooter =  new Shooter(hardwareMap);    // same as TeleOp
        ImuUtil imu      = new ImuUtil(hardwareMap);    // same as TeleOp

        telemetry.addLine("Auto Ready");
        telemetry.update();
        // after constructing imu, before waitForStart:
        imu.resetYaw();


        waitForStart();
        if (isStopRequested()) return;

        //drive.driveStraightInches(this, 1.0, 0.25);               // forward 24 inches
        //drive.turnToHeadingDegrees(this, imu, 180.0, 0.5, 0.015); // face 180°
        drive.driveReverse();
        sleep(1000);
        drive.stopAll();
        shooter.closeShoot();
        sleep(1500);
        shooter.feedOne(this);
        shooter.stop();
        //drive.driveStraightInches(this, 12.0, 0.4);               // forward 12 inches

        telemetry.addLine("Auto Done");
        telemetry.update();
        sleep(250);
    }
}
