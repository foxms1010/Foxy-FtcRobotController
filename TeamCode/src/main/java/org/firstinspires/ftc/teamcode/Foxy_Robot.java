package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Foxy_Robot {

    // This declares the four motors needed
    public DcMotor frontLeftDrive;
    public DcMotor frontRightDrive;
    public DcMotor backLeftDrive;
    public DcMotor backRightDrive;

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

        // initialize drive motors
        frontLeftDrive = hardwareMap.get(DcMotor.class, FOXY_CONFIG.HARDWARE.DRIVETRAIN.MOTORS.FRONT_LEFT);
        frontRightDrive = hardwareMap.get(DcMotor.class, FOXY_CONFIG.HARDWARE.DRIVETRAIN.MOTORS.FRONT_RIGHT);
        backLeftDrive = hardwareMap.get(DcMotor.class, FOXY_CONFIG.HARDWARE.DRIVETRAIN.MOTORS.BACK_LEFT);
        backRightDrive = hardwareMap.get(DcMotor.class, FOXY_CONFIG.HARDWARE.DRIVETRAIN.MOTORS.BACK_RIGHT);

        // set brake mode on for firm stopping behavior
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // We set the left motors in reverse which is needed for drive trains where the left
        // motors are opposite to the right ones.
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
    }
}
