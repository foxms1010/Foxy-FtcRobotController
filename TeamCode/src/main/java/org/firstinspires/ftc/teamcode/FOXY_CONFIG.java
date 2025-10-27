package org.firstinspires.ftc.teamcode;

public interface FOXY_CONFIG {
    // drive motors
    interface HARDWARE {
        interface DRIVETRAIN {
            interface MOTORS {
                String FRONT_LEFT = "motorFrontLeft";
                String FRONT_RIGHT = "motorFrontRight";
                String BACK_LEFT = "motorBackLeft";
                String BACK_RIGHT = "motorBackRight";
            }
            interface Encoders {
                String PARALLEL_LEFT = "motorFrontLeft";
                String PARALLEL_RIGHT = "motorBackRight";
                String PERPENDICULAR = "motorBackLeft";
            }
        }
        interface INTAKE {
            interface MOTORS {
                String PRIMARY = "";
            }
        }
        interface OUTTAKE {
            interface MOTORS {
                String LEFT = "";
                String RIGHT = "";
            }
        }
        interface Sensors {
            interface IMU {
                String PRIMARY = "imu";
            }
        }
    }
    interface PARAMS {
        interface DRIVETRAIN {
            double IN_PER_TICK = 0.0018304681; // 60 inches / 32778.5 ticks // deadwheel measurement with forwardPushTest
            double LATERAL_IN_PER_TICK = 0.0012007293742600262; // deadwheel measurement with lateralRampLogger
            double TRACK_WIDTH_TICKS = 7789.499465051738; // deadwheel measurement with angularRampLogger

            // feed forward parameters (in tick units)
            double kV = 0.00038289361648751954; // determined with forwardRampLogger
            double kS = 0.6564445965996808; // determined with forwardRampLogger
            double kA = 0.0001; // experimentally tuned with ManualFeedForwardTuner

            // path profile parameters (in inches)
            double MAX_WHEEL_VEL = 50;
            double MIN_PROFILE_ACCEL = -30;
            double MAX_PROFILE_ACCEL = 50;

            // turn profile parameters (in radians)
            double MAX_ANG_VEL = Math.PI; // shared with path
            double MAX_ANG_ACCEL = Math.PI;

            // path controller gains
            double AXIAL_GAIN = 8;
            double LATERAL_GAIN = 5;
            double HEADING_GAIN = 6; // shared with turn

            double AXIAL_VEL_GAIN = 0.0;
            double LATERAL_VEL_GAIN = 0.0;
            double HEADING_VEL_GAIN = 0.0; // shared with turn

            // THROTTLE CONTROL
            double MAX_POWER = 1.0; // decimal percentage
            double MAX_SPEED = 1.0; // multiplier for MAX_POWER
        }
        interface DEADWHEELS {
            double PAR_0_TICKS = -3196.521680482703; // y-pos of parallel encoder (in tick units) // measured with angularRampLogger
            double PAR_1_TICKS = 2996.083385070517;  // y-pos of parallel encoder (in tick units) // measured with angularRampLogger
            double PERP_TICKS = -3663.3660928523823; // x-pos of perpendicular encoder (in tick units) // measured with angularRampLogger


        }
    }
}
