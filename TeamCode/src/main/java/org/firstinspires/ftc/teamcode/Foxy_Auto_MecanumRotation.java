package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class Foxy_Auto_MecanumRotation extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Foxy_Robot myRobot = new Foxy_Robot(hardwareMap);
        Pose2d beginPose = new Pose2d(0, 0, 0);

        MecanumDrive drive = new MecanumDrive(myRobot, beginPose);

        waitForStart();

        while (opModeIsActive()) {
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(0, 0),
                    0.2
            ));

            // store pose for use in later TeleOp modes
            FOXY_GLOBAL_STORAGE.currentPose = drive.localizer.getPose();
        }
    }
}
