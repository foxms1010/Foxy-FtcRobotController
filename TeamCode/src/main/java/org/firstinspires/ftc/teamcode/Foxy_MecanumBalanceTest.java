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

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.lang.Math;
import java.util.ArrayList;
import java.util.List;
import java.util.Vector;

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
@TeleOp(name = "Foxy: Mecanum Balance Test", group = "Foxy")
public class Foxy_MecanumBalanceTest extends OpMode {

    Foxy_Robot myRobot;
    MecanumDrive drive;
    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();

    public static double DRIVE_POWER = FOXY_CONFIG.PARAMS.DRIVETRAIN.MAX_POWER;
    public static double FL_scalar = FOXY_CONFIG.PARAMS.DRIVETRAIN.FRONT_LEFT_SCALAR;
    public static double FR_scalar = FOXY_CONFIG.PARAMS.DRIVETRAIN.FRONT_RIGHT_SCALAR;
    public static double BL_scalar = FOXY_CONFIG.PARAMS.DRIVETRAIN.BACK_LEFT_SCALAR;
    public static double BR_scalar = FOXY_CONFIG.PARAMS.DRIVETRAIN.BACK_RIGHT_SCALAR;

    // empirical data collection
    public double maxMeasuredAngVel = 0.0;
    public double lastMeasuredAngVel = 0.0;
    public double maxMeasuredAngAccel = 0.0;
    public double currentMeasuredAngAccel = 0.0;
    public double currentMeasuredAngVel = 0.0;
    public ElapsedTime loopTime = new ElapsedTime();

    @Override
    public void init() {
        // initialize myRobot
        myRobot = new Foxy_Robot(hardwareMap);
        drive = new MecanumDrive(myRobot, new Pose2d(0,0,0));

        myRobot.imu.resetYaw();

        // Pull initial pose from GlobalStorage. If no AutoOp has set the pose, it will
        // default to 0,0,0
        Pose2d initTelePose = FOXY_GLOBAL_STORAGE.currentPose;
        telemetry.addData("Initial TeleOp Pose: ", initTelePose);
        telemetry.addData("Drive Mode: ", myRobot.driveMode);
        telemetry.update();
    }

    @Override
    public void loop() {
        telemetry.addLine("X: Fwd-Left Diag");
        telemetry.addLine("B: Fwd-Right Diag");
        telemetry.addLine("A: Rotate");
        telemetry.addLine("Y: Stop");

        TelemetryPacket packet = new TelemetryPacket();

        if (gamepad1.x) {
            // fwd-left: FR, BL
            //this.setDrivePower(0, DRIVE_POWER, DRIVE_POWER, 0);

            // use RoadRunner to drive fwd-left at 45 degrees while maintaining same heading
            // TODO: FIGURE OUT HOW TO TRANSPOSE RR FIELD COORDS TO ROBOT COORDS OR VICE VERSA
            Pose2d fwdLeftStartPose = drive.localizer.getPose();
            Vector2d fwdLeftNewVector = new Vector2d(fwdLeftStartPose.position.x + 36,
                    fwdLeftStartPose.position.y + 36);
            Action diagFwdLeftAction = drive.actionBuilder(fwdLeftStartPose)
                    .strafeToConstantHeading(fwdLeftNewVector)
                    .build();
            runningActions.add(new SequentialAction(
                    diagFwdLeftAction,
                    new SleepAction(1.0)
            ));
        }
        if (gamepad1.b) {
            // fwd-right: FL, BR
            //this.setDrivePower(DRIVE_POWER, 0, 0, DRIVE_POWER);

            // use RoadRunner to drive fwd-right at 45 degrees while maintaining same heading
            // TODO: FIGURE OUT HOW TO TRANSPOSE RR FIELD COORDS TO ROBOT COORDS OR VICE VERSA
            Pose2d fwdRightStartPose = drive.localizer.getPose();
            Vector2d fwdRightNewVector = new Vector2d(fwdRightStartPose.position.x + 36,
                                                    fwdRightStartPose.position.y - 36);
            Action diagFwdRightAction = drive.actionBuilder(fwdRightStartPose)
                    .strafeToConstantHeading(fwdRightNewVector)
                    .build();
            runningActions.add(new SequentialAction(
                    diagFwdRightAction,
                    new SleepAction(1.0)
            ));
        }

        if (gamepad1.a) {
            // rotate: -FL, FR, -BL, BR
            // this.setDrivePower(-DRIVE_POWER, DRIVE_POWER, -DRIVE_POWER, DRIVE_POWER);

            // use RoadRunner to rotate 360 and then output dX and dY
            Action rotateAction = drive.actionBuilder(drive.localizer.getPose()).turn(Math.toRadians(180)).build();
            runningActions.add(new SequentialAction(
               rotateAction,
                    new SleepAction(1.0)
            ));
        }
        if (gamepad1.y) {
            // Roadrunner: clear current running actions
            runningActions.clear();

            // stop
            this.setDrivePower(0,0,0,0);
        }

        // update running actions
        List<Action> newActions = new ArrayList<>();
        for (Action action : runningActions) {
            action.preview(packet.fieldOverlay());
            if (action.run(packet)) {
                newActions.add(action);
            }
        }
        runningActions = newActions;


        // get current vel
        currentMeasuredAngVel = drive.updatePoseEstimate().angVel;

        // calculate change in time for this loop
        double deltaT = loopTime.seconds();

        // calculate angular acceleration
        if (deltaT > 0) {
            // calculate change in vel since last loop
            double deltaV = currentMeasuredAngVel - lastMeasuredAngVel;
            currentMeasuredAngAccel = deltaV / deltaT;
        }

        // update last velocity and reset timer
        lastMeasuredAngVel = currentMeasuredAngVel;
        loopTime.reset();

        // check max values
        maxMeasuredAngVel = Math.max(maxMeasuredAngVel, currentMeasuredAngVel);
        maxMeasuredAngAccel = Math.max(maxMeasuredAngAccel, currentMeasuredAngAccel);

        // put new pose data in packet
        packet.put("x", drive.localizer.getPose().position.x);
        packet.put("y", drive.localizer.getPose().position.y);
        packet.put("heading", Math.toDegrees(drive.localizer.getPose().heading.log()));
        packet.put("IMU heading", myRobot.imu.getRobotYawPitchRollAngles().getYaw());
        packet.put("WheelVel", drive.updatePoseEstimate().linearVel);
        packet.put("AngVel", currentMeasuredAngVel);
        packet.put("MaxAngVel", maxMeasuredAngVel);
        packet.put("MaxAngVel(degrees)", Math.toDegrees(maxMeasuredAngVel));
        packet.put("AngAccel", currentMeasuredAngAccel);
        packet.put("MaxAngAccel", maxMeasuredAngAccel);
        dash.sendTelemetryPacket(packet);
    }

    public void setDrivePower(double FL, double FR, double BL, double BR) {

        myRobot.frontLeftDrive.setPower(FL * FL_scalar);
        myRobot.frontRightDrive.setPower (FR * FR_scalar);
        myRobot.backLeftDrive.setPower (BL * BL_scalar);
        myRobot.backRightDrive.setPower(BR * BR_scalar);

    }
}
