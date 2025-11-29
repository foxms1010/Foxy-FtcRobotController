/* Copyright (c) 2025 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.lang.Math;
import java.util.Locale;

/*
 * This OpMode illustrates how to program your robot to drive field relative.  This means
 * that the robot drives the direction you push the joystick regardless of the current orientation
 * of the robot.
 *
 * This OpMode assumes that you have four mecanum wheels each on its own motor named:
 *   front_left_motor, front_right_motor, back_left_motor, back_right_motor
 *
 *   and that the left motors are flipped such that when they turn clockwise the wheel moves backwards
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 *
 */
@Config
@TeleOp(name = "Foxy: Field Relative Mecanum Drive", group = "Foxy")
public class Foxy_FieldRelativeDrive extends OpMode {

    Foxy_Robot myRobot;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private AprilTagDetection desiredTag = null;
    private static final int DESIRED_TAG_ID = FOXY_CONFIG.GAME_DECODE_TARGETS_RED_ID;  // ID 24 = red target aprilTag

    private FtcDashboard dashboard = FtcDashboard.getInstance();
    private TelemetryPacket packet;

    @Override
    public void init() {
        // initialize myRobot
        myRobot = new Foxy_Robot(hardwareMap, telemetry);
        myRobot.initCamera();

        // Pull initial pose from GlobalStorage. If no AutoOp has set the pose, it will
        // default to 0,0,0
        Pose2d initTelePose = FOXY_GLOBAL_STORAGE.currentPose;
        telemetry.addData("Initial TeleOp Pose: ", initTelePose);
        telemetry.addData("Drive Mode: ", myRobot.driveMode);
        telemetry.update();

        myRobot.localizer = new ThreeDeadWheelLocalizer(
                hardwareMap,
                FOXY_CONFIG.PARAMS_DRIVETRAIN_IN_PER_TICK,
                initTelePose);
    }

    @Override
    public void loop() {
        this.packet = new TelemetryPacket();

        telemetry.addLine("A: reset Yaw");
        telemetry.addLine("Left Bumper: field-relative");
        telemetry.addLine("Right Bumper: robot-relative");

        // If you press the A button, then you reset the Yaw to be zero from the way
        // the robot is currently pointing
        if (gamepad1.a) {
            myRobot.resetYaw(myRobot.localizer);
        }
        // If you press the left bumper, set drive mode to FIELD_RELATIVE
        if (gamepad1.left_bumper) {
            myRobot.driveMode = Foxy_Robot.DRIVE_MODE.FIELD_RELATIVE;
        }

        // If you press the right bumper, set drive mode to ROBOT_RELATIVE
        if (gamepad1.right_bumper) {
            myRobot.driveMode = Foxy_Robot.DRIVE_MODE.ROBOT_RELATIVE;
        }

        if (gamepad1.y){
            autoAimToTarget();
        }

        if (gamepad1.x) {
            myRobot.webSight.update();
            desiredTag = null;
            desiredTag = myRobot.webSight.getTagBySpecificId(DESIRED_TAG_ID);
            if (desiredTag != null) {
                telemetry.addLine(String.format(Locale.US, "RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", desiredTag.ftcPose.range, desiredTag.ftcPose.bearing, desiredTag.ftcPose.elevation));
            }
        }


        telemetry.addData("Drive Mode: ", myRobot.driveMode);
        telemetry.addData("Current Heading: ", Math.toDegrees(myRobot.localizer.getPose().heading.log()));

        telemetry.addData("drive Args(fwd):", -gamepad1.left_stick_y);
        telemetry.addData("drive Args(right):", gamepad1.left_stick_x);
        telemetry.addData("drive Args (rot):", gamepad1.right_stick_x);

        if (myRobot.driveMode == Foxy_Robot.DRIVE_MODE.ROBOT_RELATIVE) {
            myRobot.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        } else {
            myRobot.driveFieldRelative(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        }
        myRobot.updatePoseEstimate();
        telemetry.update();
        dashboard.sendTelemetryPacket(this.packet);
    }


    private void autoAimToTarget() {
        myRobot.webSight.update();
        desiredTag = null;

        // Step 1: Get the list of current detections
        desiredTag = myRobot.webSight.getTagBySpecificId(DESIRED_TAG_ID);

        // Step 2: If found, calculate correction and apply power
        if (desiredTag != null) {
            // The reported bearing value is our current angular position relative to the tag
            double currentBearing = desiredTag.ftcPose.bearing;

            // Calculate the error: Current observed bearing minus the desired bearing (our 10-degree offset)
            double bearingError = currentBearing - FOXY_CONFIG.PARAMS_CTRL_AIM_LONG_TARGET_BEARING;

            // collect telemetry for graphing
            this.packet.put("currentBearing", currentBearing);
            this.packet.put("targetBearing", FOXY_CONFIG.PARAMS_CTRL_AIM_LONG_TARGET_BEARING);
            this.packet.put("bearingError", bearingError);

            // Calculate turn power using a simple proportional controller
            // Kp_TURN * bearingError = motor power adjustment
//            double turnPower = -bearingError * FOXY_CONFIG.PARAMS_CTRL_AIM_LONG_kP_TURN;
//            telemetry.addData("turn power: ", turnPower);
            telemetry.addLine(String.format(Locale.US, "\n==== (ID %d) %s", desiredTag.id, desiredTag.metadata.name));
            telemetry.addLine(String.format(Locale.US,"RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", desiredTag.ftcPose.range, desiredTag.ftcPose.bearing, desiredTag.ftcPose.elevation));

            // PID to target bearing
            myRobot.driveWithPID(0, currentBearing, 0, FOXY_CONFIG.PARAMS_CTRL_AIM_LONG_TARGET_BEARING);

            myRobot.updatePoseEstimate();

//            if (Math.abs(bearingError) > FOXY_CONFIG.PARAMS_CTRL_AIM_LONG_BEARING_DEADBAND) {
//                myRobot.drive(0, 0, turnPower);
//            } else {
//                // The reported range value is our current y position relative to the tag
//                double currentRange = desiredTag.ftcPose.range;
//
//                // Calculate the error: Current observed range minus the desired range
//                double rangeError = currentRange - FOXY_CONFIG.PARAMS_CTRL_AIM_LONG_TARGET_RANGE;
//
//                // collect telemetry for graphing
//                this.packet.put("currentRange", currentRange);
//                this.packet.put("targetRange", FOXY_CONFIG.PARAMS_CTRL_AIM_LONG_TARGET_RANGE);
//                this.packet.put("rangeError", rangeError);
//
//                // Calculate drive power using a simple proportional controller
//                // Kp_DRIVE * rangeError = motor power adjustment
//                double drivePower = rangeError * FOXY_CONFIG.PARAMS_CTRL_AIM_LONG_kP_DRIVE;
//
//                if(Math.abs(rangeError) > FOXY_CONFIG.PARAMS_CTRL_AIM_LONG_RANGE_DEADBAND) {
//                    myRobot.drive(drivePower, 0, 0);
//                }
//
//                myRobot.updatePoseEstimate();
//            }
        }
        else {
            telemetry.addLine("No aprilTag found :(");

        }
    }
}
