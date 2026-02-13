package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.common.RobotConstants;

public class ImuUtil {
    private final IMU imu;
    private final IMU.Parameters params;

    //Make sure to tune the directions when you move the control hub
     public ImuUtil(HardwareMap hw) {
        imu = hw.get(IMU.class, RobotConstants.IMU);
        params = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        ));
        imu.initialize(params);
    }

    public void resetYaw() { imu.resetYaw(); }
    public void reinit()   { imu.initialize(params); }

    public double getHeadingRad() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    IMU getRaw() { return imu; } // optional, for logging
}
