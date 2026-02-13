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
        // 1. SHOOT ONE
        // ---------------------------
        vision.faceTagUntil(this);
        sleep(1500);
        shooter.farShoot();
        sleep(1800);            // wait for flywheel to stabilize

        shooter.feedOne(this);

        // ---------------------------
        // 2. INTAKE 2nd ball into fly wheel
        // ---------------------------
        sleep(450);
        shooter.intake();
        sleep(500);
        shooter.intakeReverse();
        sleep(3000);

        shooter.feedOne(this);

        // ---------------------------
        // 3. Move off line
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
