package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystem.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystem.ImuUtil;
import org.firstinspires.ftc.teamcode.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.subsystem.VisionAlign;


@TeleOp(name="Test Robot", group="Pedro Pathing")
public class TestSubsystems extends LinearOpMode {
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

        int option = 1;
        boolean prevLeftBumper  = false;
        boolean prevRightBumper = false;
        double rpm = 0;

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y  = -gamepad1.left_stick_y;
            double x  =  gamepad1.left_stick_x;
            double rx =  gamepad1.right_stick_x;

            if (gamepad1.options) imu.resetYaw();
            double heading = imu.getHeadingRad();
            shooter.update();

            boolean fast = gamepad1.right_trigger > 0.1;
            boolean slow = gamepad1.left_trigger  > 0.1;

            // Detect rising edge (button just pressed)
            if (gamepad1.right_bumper && !prevRightBumper) {
                option++;
                if (option > 4) option = 1;
            } else if (gamepad1.left_bumper && !prevLeftBumper) {
                option--;
                if (option < 1) option = 4;
            }
        
            prevRightBumper = gamepad1.right_bumper;
            prevLeftBumper  = gamepad1.left_bumper;
        
            // Now display the current menu
            telemetry.clearAll();
            telemetry.addData("Option ", option);
            telemetry.addLine("Use bumpers to move selection");


            //Option 1: Drive train, Option 2: Shooting system, Option 3: Vision System, Option 4: Motor Test
            if (option == 1){//Drive Train

                drive.driveFieldCentric(x, y, rx, heading, slow, fast, false);

                if (gamepad1.dpad_right) drive.nudgeRight(this);
                if (gamepad1.dpad_left)  drive.nudgeLeft(this);
                if (gamepad1.dpad_up)    drive.nudgeForward(this);
                if (gamepad1.dpad_down)  drive.nudgeBack(this);

                if (gamepad1.a) drive.testDrive();


                telemetry.addLine("Drive Train");
                telemetry.addLine("========================================");
                telemetry.addLine("Joysticks - Left: Forward & Backward Right: Turning");
                telemetry.addLine("Left  Trigger  - Slow Mode");
                telemetry.addLine("Right Trigger  - Fast Mode");
                telemetry.addLine("Gampad A  - Test Drive");
                telemetry.addLine("Gampad B  -");
                telemetry.addLine("Gampad X  -");
                telemetry.addLine("Gampad Y  -");
                telemetry.addLine("Dpads - Scoot");
                telemetry.update();
            }
            else if (option == 2){

                if (gamepad1.right_trigger > 0.1) shooter.farShoot();
                else                              shooter.stop();

                if (gamepad1.left_trigger > 0.1) shooter.closeShoot();
                else                              shooter.stop();

                if (gamepad1.left_bumper) shooter.intakeFW();

                if (gamepad1.b) shooter.intake();
                else            shooter.stopIntake();

                if (gamepad1.a) shooter.feedOne(this);

                if (gamepad1.x){
                    shooter.customShoot(rpm, this);
                }

                if (gamepad1.dpad_up) {
                    rpm = rpm + 10.0;
                    sleep(100);
                }
                if (gamepad1.dpad_down){
                    rpm = rpm - 10.0;
                    sleep(100);
                }

                if (gamepad1.y){
                    shooter.shootByDistance(vision.getDistance(), this);
                }


                telemetry.addLine("Shooting System");
                telemetry.addLine("========================================");
                telemetry.addLine("Joysticks -");
                telemetry.addLine("Left  Trigger  - Close Shot");
                telemetry.addLine("Right Trigger  - Far Shot");
                telemetry.addLine("Left  Bumper  - Reverse Spin");
                telemetry.addLine("Gampad A  - Feed a ball via servo");
                telemetry.addLine("Gampad B  - Intake (If installed)");
                telemetry.addLine("Gampad X  - Custom RPM shoot");
                telemetry.addLine("Gampad Y  - Shoot By Distance");
                telemetry.addLine("Dpad Up   - Power +10 rpm");
                telemetry.addLine("Dpad Down - Power -10 rpm");
                telemetry.addLine("========================================");
                telemetry.addData("Flywheel RPM:", shooter.getFlywheelRpm());
                telemetry.addData("CustomRPM:", rpm);
                telemetry.addData("Distance:", vision.getDistance());
                telemetry.update();
            }
            else if (option == 3){

                // Ensure VisionAlign + Limelight is created at top of file
                if (vision == null) {
                    vision = new VisionAlign(drive, imu);
                    vision.start(hardwareMap);
                }

                LLResult r = vision.latest();

                if (r != null && r.isValid()) {
                    telemetry.addLine("VISION SYSTEM (LL DEBUG)");
                    telemetry.addData("VALID", true);
                    telemetry.addData("tx", "%.2f", r.getTx());
                    telemetry.addData("ty", "%.2f", r.getTy());
                    telemetry.addData("ta", "%.2f", r.getTa());
                    telemetry.addData("Pipeline", r.getPipelineIndex());
                } else {
                    telemetry.addLine("VISION SYSTEM (LL DEBUG)");
                    telemetry.addData("VALID", false);
                }

                if (r != null && r.isValid()) {
                    for (LLResultTypes.FiducialResult f : r.getFiducialResults()) {
                        telemetry.addData("Tag Seen", f.getFiducialId());
                    }
                }

                if (gamepad1.a) vision.faceTagUntil(this);
                if (gamepad1.b) vision.aimAndApproachStepRobotCentric();

                telemetry.addLine("Vison System");
                telemetry.addLine("!!!NOT IMPLEMENTED YET!!!");
                telemetry.addLine("========================================");
                telemetry.addLine("Joysticks - ");
                telemetry.addLine("Left  Trigger  -");
                telemetry.addLine("Right Trigger  -");
                telemetry.addLine("Gampad A  - Turn to center the tag");
                telemetry.addLine("Gampad B  - Turn + creep forward toward standoff");
                telemetry.addLine("Gampad X  -");
                telemetry.addLine("Gampad Y  -");
                telemetry.addLine("Dpads -");
                telemetry.update();
            }
            else if (option == 4){

                double testPower = 0.2;

                if (gamepad1.a)  drive.fr.setPower(testPower);
                else             drive.fr.setPower(0);

                if (gamepad1.b)  drive.br.setPower(testPower);
                else             drive.br.setPower(0);

                if (gamepad1.x)  drive.fl.setPower(testPower);
                else             drive.fl.setPower(0);

                if (gamepad1.y)  drive.bl.setPower(testPower);
                else             drive.bl.setPower(0);

                if (gamepad1.dpad_up)    shooter.closeShoot();
                else                     shooter.stop();    

                if (gamepad1.dpad_down)  shooter.intake();
                else                     shooter.stopIntake();


                telemetry.addLine("Motor Test");
                telemetry.addLine("========================================");
                telemetry.addLine("Joysticks - Left: Forward & Backward Right: Turning");
                telemetry.addLine("Left  Trigger  -");
                telemetry.addLine("Right Trigger  -");
                telemetry.addLine("Gampad A  - Front Right");
                telemetry.addLine("Gampad B  - Back Right");
                telemetry.addLine("Gampad X  - Front Left");
                telemetry.addLine("Gampad Y  - Back Left");
                telemetry.addLine("Dpads - ^ Shooter | v Intake");
                telemetry.update();
            }
            else{
                drive.stopAll();
                shooter.stop();
                shooter.stopIntake();

                telemetry.addLine("There has been an error, please restart the robot");
                telemetry.addLine("And then tell Jayden how you made the error happen :)");
                telemetry.update();

            }
        }
    }
}
