package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

@Autonomous
public class Foxy_Auto_RRTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(0, 0, 0);
        Foxy_Robot myRobot = new Foxy_Robot(hardwareMap);
        MecanumDrive drive = new MecanumDrive(myRobot, beginPose);

        waitForStart();

        Actions.runBlocking(
                drive.actionBuilder(beginPose)
//                        .lineToX(15)
//                        .turn(Math.toRadians(90))
//                        .lineToY(15)
//                        .turn(Math.toRadians(90))
//                        .lineToX(0)
//                        .turn(Math.toRadians(90))
//                        .lineToY(0)
//                        .turn(Math.toRadians(90))
                        .splineTo(new Vector2d(25,-25),Math.toRadians(-90))
                        .build());

        // store pose for use in later TeleOp modes
        FOXY_GLOBAL_STORAGE.currentPose = drive.localizer.getPose();
    }
}
