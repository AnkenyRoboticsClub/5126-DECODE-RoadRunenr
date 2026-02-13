package org.firstinspires.ftc.teamcode.RRautos;

import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Pose2d;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.AutoDrive;
import org.firstinspires.ftc.teamcode.subsystem.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystem.ImuUtil;
import org.firstinspires.ftc.teamcode.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.subsystem.VisionAlign;

@Autonomous(name = "Blue Far", group = "RoadRunnerAuto")
public class BlueFar extends LinearOpMode{

    @Override
    public void runOpMode() throws InterruptedException {

        AutoDrive autoDrive = new AutoDrive(hardwareMap);
        ImuUtil imu = new ImuUtil(hardwareMap);
        Shooter shooter = new Shooter(hardwareMap);
        DriveTrain teleDrive = new DriveTrain(hardwareMap); // used for LL movement
        VisionAlign vision = new VisionAlign(teleDrive, imu);
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, Math.toRadians(0)));

        imu.resetYaw();
        vision.start(hardwareMap);

        telemetry.addLine("Auto Ready");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;
    }
}
