package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class Foxy_Auto_RRTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(0, 0, 0);

        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        waitForStart();

        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .lineToX(30)
                        .turn(Math.toRadians(90))
                        .lineToY(30)
                        .turn(Math.toRadians(90))
//                        .lineToX(0)
//                        .turn(Math.toRadians(90))
//                        .lineToY(0)
//                        .turn(Math.toRadians(90))
                        .build());

        // store pose for use in later TeleOp modes
        FOXY_GLOBAL_STORAGE.currentPose = drive.localizer.getPose();
    }
}
