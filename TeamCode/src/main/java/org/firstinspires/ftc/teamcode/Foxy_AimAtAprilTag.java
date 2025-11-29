package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.mechanisms.Foxy_Camera;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Autonomous
public class Foxy_AimAtAprilTag extends OpMode {
    Foxy_Robot myRobot;
    Pose2d beginPose = new Pose2d(0, 0, 0);

    MecanumDrive drive;

    @Override
    public void init(){
        myRobot = new Foxy_Robot(hardwareMap, telemetry);

        // initialize the camera and processor
        myRobot.initCamera();

        drive = new MecanumDrive(hardwareMap, beginPose);

    }

    @Override
    public void loop(){
        //update the vision portal
        myRobot.webSight.update();
        AprilTagDetection detectionId24 = myRobot.webSight.getTagBySpecificId(24);
        myRobot.webSight.displayDetectionTelemetry(detectionId24);

  /*      // get range and bearing
        if (detectionId24 != null) {
            double targetRange = detectionId24.ftcPose.range;
            double targetBearing = detectionId24.ftcPose.bearing;

            // rotate to bring bearing to 0 +/- 2 degrees
            if (Math.abs(targetBearing) > FOXY_CONFIG.PARAMS.DRIVETRAIN.BEARING_DEADBAND) {
                Actions.runBlocking(
                        drive.actionBuilder(beginPose)
                                .turn(Math.toRadians(-targetBearing))
                                .build());
            }
        }*/
    }
}
