package org.firstinspires.ftc.teamcode.common;

public class RobotConstants {
    private RobotConstants() {}

    // Drive speed scales, power # is from 0 -> 1, 0=0% and 1=100%
    public static final double TURN_SPEED          = 0.7; //Prob should stay the same as default speed, honestly idk
    public static final double MOVING_SPEED_SLOW   = 0.4;
    public static final double MOVING_SPEED        = 0.6; //Default Speed
    public static final double MOVING_SPEED_FAST   = 1.0; //Usualy max power

    // Shooter tuning
    public static final double FLY_SPEED_SHOOT   = 1.0;  // forward shoot speed
    public static final double FLY_SPEED_REVERSE = -0.75; // gentle reverse for unjam

    public static final double FLY_CLOSE_RPM = 670; //A guess
    public static final double FLY_FAR_RPM = 1000; //A guess

    // Kicker positions (tune these!)
    public static final double KICK_RETRACT = -0.10; // Down / hidden position
    public static final double KICK_EXTEND  = 0.30; // Extended to eject balls
    public static final long   KICK_TIME_MS = 100;  // How long to be extended for in ms

    // Nudge (“scootch”)
    public static final double SCOOTCH_POWER       = 0.5;   // tune for more powerfull scootch
    public static final long   SCOOTCH_DURATION_MS = 200;   // tune for longer or shorter scootch

    // Motor names (edit if your config names change)
    public static final String M_FL = "motor3";
    public static final String M_FR = "motor2";//Good
    public static final String M_BL = "motor1";
    public static final String M_BR = "motor0";//Good

    public static final String M_FLY = "motor4";//OG4
    public static final String S_KICK = "kickServo";
    public static final String M_INTAKE = "motorI";

    public static final String S_INTAKE = "servoI";

    public static final String PINPOINT = "pinpoint";
    
    //ViperCode V
    //public static final String M_VL = "motorVL";
    //public static final String M_VR = "motorVR";

    // IMU name
    public static final String IMU = "imu";

    public static final double TICKS_PER_MOTOR_REV = 28.0 * 4.0; // = 112
    public static final double GEAR_RATIO = 48.0 / 10.0;  // 4.8 : 1
    public static final double WHEEL_DIAMETER_IN = 4.0944882; //or 104mm i think
    public static final double WHEEL_CIRCUMFERENCE_IN =
            Math.PI * WHEEL_DIAMETER_IN;
    public static final double TICKS_PER_INCH =
            (TICKS_PER_MOTOR_REV * GEAR_RATIO) / WHEEL_CIRCUMFERENCE_IN;
    public static int inchesToTicks(double inches) {
        return (int) Math.round(inches * TICKS_PER_INCH);
    }





    // -------------------- LIMELIGHT CONSTANTS --------------------

    // Device name from RC config
    public static final String LL_DEVICE_NAME = "limelight";

    // Tuning multipliers
    public static final double LL_K_TURN      = 0.035;   // turning gain
    public static final double LL_K_FORWARD   = 0.25;    // forward gain

    // Minimum motor power so robot “breaks static friction”
    public static final double LL_MIN_TURN    = 0.17;
    public static final double LL_MIN_FORWARD = 0.20;

    // Maximum motor speeds for LL control
    public static final double LL_MAX_TURN    = 0.35;
    public static final double LL_MAX_FORWARD = 0.35;

    // TA value when robot is at perfect shooting distance
    // ★ YOU MUST UPDATE THIS using LLDebug ★
    public static double LL_TARGET_AREA = 1.45;

    // Tolerances for stopping
    public static final double LL_AIM_TOL_DEG      = 1.5;
    public static final double LL_APPROACH_TOL_TA  = 0.05;

    // Auto alignment timeouts
    public static final double LL_ALIGN_TIMEOUT_S    = 2.0;
    public static final double LL_APPROACH_TIMEOUT_S = 3.0;

    double limelightLensHeightInches = 10.53;
    double goalHeightInches = 29.5;
}
