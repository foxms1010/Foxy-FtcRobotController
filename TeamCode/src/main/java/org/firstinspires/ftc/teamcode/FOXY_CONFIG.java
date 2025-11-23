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
            interface ENCODERS {
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
        interface SENSORS {
            interface IMU {
                String PRIMARY = "imu";
            }
        }
    }
    interface PARAMS {
        interface DRIVETRAIN {
            double IN_PER_TICK = 0.0018602921;  // deadwheel measurement with forwardPushTest
            double LATERAL_IN_PER_TICK = 0.001296650302183416; // deadwheel measurement with lateralRampLogger
            double TRACK_WIDTH_TICKS = 7730.066710732239; // deadwheel measurement with angularRampLogger

            // feed forward parameters (in tick units)
            double kV = 0.00038436829256315367; // determined with forwardRampLogger
            double kS = 0.6127587708109248; // determined with forwardRampLogger
            double kA = 0.0001; // experimentally tuned with ManualFeedForwardTuner

            // path profile parameters (in inches)
            double MAX_WHEEL_VEL = 50;
            double MIN_PROFILE_ACCEL = -30;
            double MAX_PROFILE_ACCEL = 50;

            // MEASURED turn profile parameters (in radians)
            double MAX_PHYSICAL_ANG_VEL = 3.14; // trial/error with FoxyMecanumBalanceTest
            double MAX_PHYSICAL_ANG_ACCEL = 3.14; // trial/error with FoxyMecanumBalanceTest

            // turn profile parameters (in radians)
            double MAX_ANG_VEL = (1.0 * MAX_PHYSICAL_ANG_VEL); // throttle down vel
            double MAX_ANG_ACCEL = (1.0 * MAX_PHYSICAL_ANG_ACCEL); // throttle down accel

            // path controller gains
            double AXIAL_GAIN = 4;
            double LATERAL_GAIN = 5;
            double HEADING_GAIN = 4; // shared with turn

            double AXIAL_VEL_GAIN = 0.0;
            double LATERAL_VEL_GAIN = 0.0;
            double HEADING_VEL_GAIN = 0.0; // shared with turn

            // THROTTLE CONTROL
            double MAX_POWER = 1.0; // decimal percentage
            double MAX_SPEED = 1.0; // multiplier for MAX_POWER

            // WHEEL BALANCING
            double FRONT_LEFT_SCALAR = 1; // multiplier for front left drive motor
            double FRONT_RIGHT_SCALAR = 1; // multiplier for front right
            double BACK_LEFT_SCALAR = 1; // multiplier for back left
            double BACK_RIGHT_SCALAR = 1; //multiplier for back right
        }
        interface DEADWHEELS {
            double PAR_0_TICKS = -1696.9292036766128; // y-pos of parallel encoder (in tick units) // measured with angularRampLogger
            double PAR_1_TICKS = 1560.4088930935536;  // y-pos of parallel encoder (in tick units) // measured with angularRampLogger
            double PERP_TICKS = 145.4091595750698; // x-pos of perpendicular encoder (in tick units) // measured with angularRampLogger


        }
    }
}
