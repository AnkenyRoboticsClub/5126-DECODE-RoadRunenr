package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystem.AutoDrive;

@Autonomous(name = "Drive Off Line")
public class DriveOffLine extends LinearOpMode {
    @Override
    public void runOpMode() {
        AutoDrive drive = new AutoDrive(hardwareMap); // same as TeleOp

        telemetry.addLine("Auto Ready");
        telemetry.update();
        
        waitForStart();
        if (isStopRequested()) return;

        drive.driveReverse();
        sleep(2000);
        drive.stopAll();
    
        telemetry.addLine("Auto Done");
        telemetry.update();
        sleep(250);
    }
}
