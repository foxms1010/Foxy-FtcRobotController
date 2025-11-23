package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

public class Foxy_Robot {

    // This declares the four motors needed
    public DcMotorEx frontLeftDrive;
    public DcMotorEx frontRightDrive;
    public DcMotorEx backLeftDrive;
    public DcMotorEx backRightDrive;

    public IMU imu;

    public HardwareMap hardwareMap;

    // RoadRunner localizer for odometry
    public ThreeDeadWheelLocalizer localizer;

    public enum DRIVE_MODE {
        ROBOT_RELATIVE,
        FIELD_RELATIVE;
    }

    public DRIVE_MODE driveMode = DRIVE_MODE.FIELD_RELATIVE;

    // This declares the IMU needed to get the current direction the robot is facing
    //IMU imu;

    public Foxy_Robot(HardwareMap hardwareMap) {

        // set the hardwareMap
        this.hardwareMap = hardwareMap;

        // initialize drive motors
        frontLeftDrive = hardwareMap.get(DcMotorEx.class, FOXY_CONFIG.HARDWARE.DRIVETRAIN.MOTORS.FRONT_LEFT);
        frontRightDrive = hardwareMap.get(DcMotorEx.class, FOXY_CONFIG.HARDWARE.DRIVETRAIN.MOTORS.FRONT_RIGHT);
        backLeftDrive = hardwareMap.get(DcMotorEx.class, FOXY_CONFIG.HARDWARE.DRIVETRAIN.MOTORS.BACK_LEFT);
        backRightDrive = hardwareMap.get(DcMotorEx.class, FOXY_CONFIG.HARDWARE.DRIVETRAIN.MOTORS.BACK_RIGHT);

        // set brake mode on for firm stopping behavior
        frontLeftDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // We set the left motors in reverse which is needed for drive trains where the left
        // motors are opposite to the right ones.
        backLeftDrive.setDirection(DcMotorEx.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotorEx.Direction.REVERSE);

        // initialize the imu
        imu = hardwareMap.get(IMU.class, FOXY_CONFIG.HARDWARE.SENSORS.IMU.PRIMARY);
    }
}
