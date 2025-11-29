package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class FOXY_CONFIG {
    // debug or competition mode
    public static boolean MODE_DEBUG = false; // set to 'false' for COMPETITION

    // target April Tags for DECODE season 2025-2026
    public static int GAME_DECODE_TARGETS_RED_ID = 24;
    public static int GAME_DECODE_TARGETS_BLUE_ID = 20;

    // drive motors
    public static String HW_DRIVE_MOTORS_FRONT_LEFT = "motorFrontLeft";
    public static String HW_DRIVE_MOTORS_FRONT_RIGHT = "motorFrontRight";
    public static String HW_DRIVE_MOTORS_BACK_LEFT = "motorBackLeft";
    public static String HW_DRIVE_MOTORS_BACK_RIGHT = "motorBackRight";

    // drive encoders (deadwheels)
    public static String HW_DRIVE_ENCODERS_PARALLEL_LEFT = "motorFrontLeft";
    public static String HW_DRIVE_ENCODERS_PARALLEL_RIGHT = "motorBackRight";
    public static String HW_DRIVE_ENCODERS_PERPENDICULAR = "motorBackLeft";

    // intake motor
    public static String HW_INTAKE_MOTORS_PRIMARY = "";

    // outtake motors
    public static String HW_OUTTAKE_MOTORS_LEFT = "";
    public static String HW_OUTTAKE_MOTORS_RIGHT = "";

    // sensors - gyro
    public static String HW_SENSORS_IMU_PRIMARY = "imu";

    // sensors - camera specs
    public static final CameraSpecs WEB_SIGHT = new CameraSpecs(
            "webSight", 4.0, 225, 941.178, 941.178, 484.54, 291.576 );

    public static final CameraSpecs FOX_RAY_VISION = new CameraSpecs(
            "foxRayVision", 4.0, 225, 937.784, 937.784, 365.701, 217.901 );

    public static final CameraSpecs FTC_DEFAULTS = new CameraSpecs(
            "ftcDefaults", 4.0, 225, 822.317, 822.317, 319.495, 242.502 );

    // sensors - camera currently installed on robot
    public static CameraSpecs ACTIVE_CAMERA = WEB_SIGHT;

    public static double PARAMS_DRIVETRAIN_IN_PER_TICK = 0.0018304681; // 60 inches / 32778.5 ticks // deadwheel measurement with forwardPushTest
    public static double PARAMS_DRIVETRAIN_LATERAL_IN_PER_TICK = 0.0012007293742600262; // deadwheel measurement with lateralRampLogger
    public static double PARAMS_DRIVETRAIN_TRACK_WIDTH_TICKS = 7789.499465051738; // deadwheel measurement with angularRampLogger

    // feed forward parameters (in tick units)
    public static double PARAMS_DRIVETRAIN_kV = 0.00038289361648751954; // determined with forwardRampLogger
    public static double PARAMS_DRIVETRAIN_kS = 0.6564445965996808; // determined with forwardRampLogger
    public static double PARAMS_DRIVETRAIN_kA = 0.0001; // experimentally tuned with ManualFeedForwardTuner

    // path profile parameters (in inches)
    public static double PARAMS_DRIVETRAIN_MAX_WHEEL_VEL = 50;
    public static double PARAMS_DRIVETRAIN_MIN_PROFILE_ACCEL = -30;
    public static double PARAMS_DRIVETRAIN_MAX_PROFILE_ACCEL = 50;

    // turn profile parameters (in radians)
    public static double PARAMS_DRIVETRAIN_MAX_ANG_VEL = Math.PI; // shared with path
    public static double PARAMS_DRIVETRAIN_MAX_ANG_ACCEL = Math.PI;

    // path controller gains
    public static double PARAMS_DRIVETRAIN_AXIAL_GAIN = 8;
    public static double PARAMS_DRIVETRAIN_LATERAL_GAIN = 5;
    public static double PARAMS_DRIVETRAIN_HEADING_GAIN = 6; // shared with turn

    public static double PARAMS_DRIVETRAIN_AXIAL_VEL_GAIN = 0.0;
    public static double PARAMS_DRIVETRAIN_LATERAL_VEL_GAIN = 0.0;
    public static double PARAMS_DRIVETRAIN_HEADING_VEL_GAIN = 0.0; // shared with turn

    // THROTTLE CONTROL
    public static double PARAMS_DRIVETRAIN_MAX_POWER = 1.0; // decimal percentage
    public static double PARAMS_DRIVETRAIN_MAX_SPEED = 1.0; // multiplier for MAX_POWER

    // deadwheels
    public static double PARAMS_DEADWHEELS_PAR_0_TICKS = -3196.521680482703; // y-pos of parallel encoder (in tick units) // measured with angularRampLogger
    public static double PARAMS_DEADWHEELS_PAR_1_TICKS = 2996.083385070517; // y-pos of parallel encoder (in tick units) // measured with angularRampLogger
    public static double PARAMS_DEADWHEELS_PERP_TICKS = -3663.3660928523823; // x-pos of perpendicular encoder (in tick units) // measured with angularRampLogger

    // controller - aim long - bearing
    public static double PARAMS_CTRL_AIM_LONG_TARGET_BEARING = 14.4; // degrees
    public static double PARAMS_CTRL_AIM_LONG_kP_TURN = 0.2; // proportional gain for turning (reactive adjustment)
    public static double PARAMS_CTRL_AIM_LONG_kI_TURN = 0; // integral gain for turning (eliminate steady state error)
    public static double PARAMS_CTRL_AIM_LONG_kD_TURN = 0; // derivative gain for turning (dampen oscillation, provide braking)
    public static double PARAMS_CTRL_AIM_LONG_MAX_kI_TURN_SUM = 20; // limit integral sum to prevent runaway power when motors are maxed
    public static double PARAMS_CTRL_AIM_LONG_BEARING_DEADBAND = 1; // degrees

    // controller - aim long - range
    public static double PARAMS_CTRL_AIM_LONG_TARGET_RANGE = 118.2; // inches
    public static double PARAMS_CTRL_AIM_LONG_kP_DRIVE = 0.1; // proportional gain for driving to target range
    public static double PARAMS_CTRL_AIM_LONG_kI_DRIVE = 0; // integral gain for driving to target (eliminate steady state error)
    public static double PARAMS_CTRL_AIM_LONG_kD_DRIVE = 0; // derivative gain for driving to target (dampen oscillation, provide braking)
    public static double PARAMS_CTRL_AIM_LONG_MAX_kI_DRIVE_SUM = 10; // limit integral sum to prevent runaway power when motors are maxed
    public static double PARAMS_CTRL_AIM_LONG_RANGE_DEADBAND = 1; // inches

    // *******************************************************
    // *****                                            ******
    // ***** NOTHING BELOW HERE SHOULD NEED TO BE TUNED ******
    // *****                                            ******
    // *******************************************************
    // sensors - configurable camera specs
    public static String HW_SENSORS_CAMERA_NAME = ACTIVE_CAMERA.NAME;
    public static double HW_SENSORS_CAMERA_EXPOSURE = ACTIVE_CAMERA.EXPOSURE;
    public static double HW_SENSORS_CAMERA_GAIN = ACTIVE_CAMERA.GAIN;
    public static double HW_SENSORS_CAMERA_FX = ACTIVE_CAMERA.FX;
    public static double HW_SENSORS_CAMERA_FY = ACTIVE_CAMERA.FY;
    public static double HW_SENSORS_CAMERA_CX = ACTIVE_CAMERA.CX;
    public static double HW_SENSORS_CAMERA_CY = ACTIVE_CAMERA.CY;

    public static class CameraSpecs {
        public String NAME;
        public double EXPOSURE;
        public double GAIN;
        public double FX;
        public double FY;
        public double CX;
        public double CY;

        public CameraSpecs(String name, double exposure, double gain, double fx, double fy, double cx, double cy) {
            this.NAME = name;
            this.EXPOSURE = exposure;
            this.GAIN = gain;
            this.FX = fx;
            this.FY = fy;
            this.CX = cx;
            this.CY = cy;
        }
    }
}
