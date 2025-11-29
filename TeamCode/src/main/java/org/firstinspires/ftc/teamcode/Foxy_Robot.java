package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.mechanisms.Foxy_Camera;

public class Foxy_Robot {

    // This declares the four motors needed
    public DcMotor frontLeftDrive;
    public DcMotor frontRightDrive;
    public DcMotor backLeftDrive;
    public DcMotor backRightDrive;

    // RoadRunner localizer for odometry
    public ThreeDeadWheelLocalizer localizer;

    // webcam
    public CameraName webCamName;
    public Foxy_Camera webSight;

    // PID state variables
    private double pidTurnPrevError = 0.0; // error from last cycle (used for derivative)
    private double pidTurnIntegralSum = 0.0; // accumulation of steady state error
    private double pidRangePrevError = 0.0; // error from last cycle (used for derivative)
    private double pidRangeIntegralSum = 0.0; // accumulation of steady state error
    private double pidCycleTime = 0.0; // time elapsed since the last loop iteration
    private long pidLastTime = System.currentTimeMillis();

    public HardwareMap hardwareMap;
    public Telemetry telemetry;

    public enum DRIVE_MODE {
        ROBOT_RELATIVE,
        FIELD_RELATIVE;
    }

    public DRIVE_MODE driveMode = DRIVE_MODE.FIELD_RELATIVE;

    // This declares the IMU needed to get the current direction the robot is facing
    //IMU imu;

    public Foxy_Robot(HardwareMap hardwareMap, Telemetry telemetry) {

        // show the current FOXY_CONFIG mode
        String strConfigMode = FOXY_CONFIG.MODE_DEBUG ? "DEBUG" : "COMPETITION";
        if (FOXY_CONFIG.MODE_DEBUG) {
            telemetry.addLine("********* WARNING *********");
            telemetry.addLine("********* WARNING *********");
        }
        telemetry.addLine("*** Config Mode is: " + strConfigMode + " ***");

        // initialize drive motors
        frontLeftDrive = hardwareMap.get(DcMotor.class, FOXY_CONFIG.HW_DRIVE_MOTORS_FRONT_LEFT);
        frontRightDrive = hardwareMap.get(DcMotor.class, FOXY_CONFIG.HW_DRIVE_MOTORS_FRONT_RIGHT);
        backLeftDrive = hardwareMap.get(DcMotor.class, FOXY_CONFIG.HW_DRIVE_MOTORS_BACK_LEFT);
        backRightDrive = hardwareMap.get(DcMotor.class, FOXY_CONFIG.HW_DRIVE_MOTORS_BACK_RIGHT);

        // set brake mode on for firm stopping behavior
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // We set the left motors in reverse which is needed for drive trains where the left
        // motors are opposite to the right ones.
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        // get camera
        webCamName = hardwareMap.get(CameraName.class, FOXY_CONFIG.HW_SENSORS_CAMERA_NAME);

        // store hwMap and telemetry
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
    }

    public void initCamera() {
        this.webSight = new Foxy_Camera();
        this.webSight.init(this.webCamName, this.hardwareMap, this.telemetry);
    }

    // Thanks to FTC16072 for sharing this code!!
    public void drive(double forward, double right, double rotate) {

        // This calculates the power needed for each wheel based on the amount of forward,
        // strafe right, and rotate
        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backRightPower = forward + right - rotate;
        double backLeftPower = forward - right + rotate;

        double maxPower = FOXY_CONFIG.PARAMS_DRIVETRAIN_MAX_POWER;
        double maxSpeed = FOXY_CONFIG.PARAMS_DRIVETRAIN_MAX_SPEED;  // make this slower for outreaches

        // This is needed to make sure we don't pass > 1.0 to any wheel
        // It allows us to keep all of the motors in proportion to what they should
        // be and not get clipped
        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));

        // We multiply by maxSpeed so that it can be set lower for outreaches
        // When a young child is driving the robot, we may not want to allow full
        // speed.
        this.frontLeftDrive.setPower(maxSpeed * (frontLeftPower / maxPower));
        this.frontRightDrive.setPower(maxSpeed * (frontRightPower / maxPower));
        this.backLeftDrive.setPower(maxSpeed * (backLeftPower / maxPower));
        this.backRightDrive.setPower(maxSpeed * (backRightPower / maxPower));
    }

    // This routine drives the robot field relative
    public void driveFieldRelative(double forward, double right, double rotate) {
        // First, convert direction being asked to drive to polar coordinates
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);

        // Second, rotate angle by the angle the robot is pointing
        theta = AngleUnit.normalizeRadians(theta -
                this.localizer.getPose().heading.log());
        //  imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        // Third, convert back to cartesian
        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        // Finally, call the drive method with robot relative forward and right amounts
        drive(newForward, newRight, rotate);
    }

    // drive with PID control to targetRange and targetBearing from current spot and heading
    public void driveWithPID (double currentRange, double currentBearing, double targetRange, double targetBearing) {
        // PID INFO:
        //
        //   P - responds to current error
        //   I - responds to accumulated past error (eliminates steady state error)
        //   D - resopnds to rate of change of error (dampens oscillation and provides braking)
        //
        //   motor_power_P = kP * current_error (tip: if overshooting target, kP is too high. if aims too slowly, too low)
        //   motor_power_I = kI * integral_sym (tip: set max limit to prevent infinite growth when motors at max power)
        //   motor_power_D = kD * error_derivative (tip: highly sensitive to sensor noise. may need a low-pass filter)
        //
        //   Tuning: (Ziegler-Nichols Method)
        //     1. Tune kP - set kI and kD to 0. Increase kP from 0 until system oscillates at constant,
        //                  sustained rate. Call this kU (ulimate gain). Measure period of oscillation (tU).
        //     2. Calculate PID Gains:
        //           kP = 0.6 * kU
        //           kI = 1.2 * kU / tU
        //           kD = 0.075 * kU * tU
        //     3. Fine Tuning:
        //          - too slow? increase kP
        //          - overshooting? increase kD
        //          - consistent error at end? increase kI

        // calculate cycle time
        long currentTime = System.currentTimeMillis();

        // let's work in seconds
        this.pidCycleTime = (currentTime - pidLastTime) / 1000.0;
        this.pidLastTime = currentTime; // store currentTime for next loop iteration

        // get Bearing error
        double turnCurrentError = targetBearing - currentBearing;

        // check if we are in range
        if (Math.abs(turnCurrentError) > FOXY_CONFIG.PARAMS_CTRL_AIM_LONG_BEARING_DEADBAND) {

            // get integral sum, capped at MAX config setting
            this.pidTurnIntegralSum += turnCurrentError * this.pidCycleTime;
            this.pidTurnIntegralSum = Math.max(-FOXY_CONFIG.PARAMS_CTRL_AIM_LONG_MAX_kI_TURN_SUM,
                    Math.min(FOXY_CONFIG.PARAMS_CTRL_AIM_LONG_MAX_kI_TURN_SUM, this.pidTurnIntegralSum));

            // get error-derivative
            double turnErrorDerivative = 0.0;
            if (this.pidCycleTime > 0) {
                turnErrorDerivative = (turnCurrentError - this.pidTurnPrevError) / this.pidCycleTime;
            }

            // collect, p-i-d terms
            double pTerm = FOXY_CONFIG.PARAMS_CTRL_AIM_LONG_kP_TURN * turnCurrentError;
            double iTerm = FOXY_CONFIG.PARAMS_CTRL_AIM_LONG_kI_TURN * this.pidTurnIntegralSum;
            double dTerm = FOXY_CONFIG.PARAMS_CTRL_AIM_LONG_kD_TURN * turnErrorDerivative;

            // calculate turn power
            double turnPower = pTerm + iTerm + dTerm;

            // now call standard drive method to set power with throttles in place
            this.drive(0, 0, turnPower);

            // update state variables
            this.pidTurnPrevError = turnCurrentError;
        }
    }

    public void resetYaw(Localizer localizer) {
        Pose2d currentPose = localizer.getPose();

        // set new pose with heading of 0, but same x, y
        Pose2d newPose = new Pose2d(currentPose.position, 0);
        localizer.setPose(newPose);
    }

    public PoseVelocity2d updatePoseEstimate() {
        PoseVelocity2d vel = this.localizer.update();

        return vel;
    }

}
