package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.tfod.Timer;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.storage.OpStorage;
import org.firstinspires.ftc.teamcode.vision.EasyOpenCVExample;
import org.firstinspires.ftc.teamcode.vision.StoneOrientationExample;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive.threadState;

@Autonomous(name = "Auto2", group = "Concept")
public class Auto2 extends LinearOpMode {

    OpenCvCamera webcam;
    StoneOrientationExample.StoneOrientationAnalysisPipeline pipeline;
    ElapsedTime timerA = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Thread externalThread = new SampleMecanumDrive.ExternalDriveThread(hardwareMap, drive);

        Pose2d startPose = new Pose2d(0, 10, Math.toRadians(0));

        drive.setPoseEstimate(startPose);

        drive.resetArmPosition();

        drive.setGrabberPosition(0);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Cam"), cameraMonitorViewId);

        // Open async and start streaming inside opened callback
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);

                pipeline = new StoneOrientationExample.StoneOrientationAnalysisPipeline();
                webcam.setPipeline(pipeline);
            }
        });

        // Tell telemetry to update faster than the default 250ms period :)
        telemetry.setMsTransmissionInterval(20);

        waitForStart();

        if (isStopRequested()) return;

        //Autonomous Motion Program Below

        int targetZoneX = 0;
        int targetZoneY = 0;



        drive.turn(Math.toRadians(-10));

        timerA.reset();

        while (opModeIsActive() && pipeline.position == StoneOrientationExample.StoneOrientationAnalysisPipeline.Rings.NULL && timerA.time() < 5)
        {
            // Don't burn an insane amount of CPU cycles in this sample because
            // we're not doing anything else
            sleep(20);

            // Figure out which stones the pipeline detected, and print them to telemetry
            ArrayList<StoneOrientationExample.StoneOrientationAnalysisPipeline.AnalyzedStone> stones = pipeline.getDetectedStones();
            StoneOrientationExample.StoneOrientationAnalysisPipeline.AnalyzedStone stone = null;
            double stoneMax = 0;

            for (StoneOrientationExample.StoneOrientationAnalysisPipeline.AnalyzedStone analyzedStone : stones) {
                if ((analyzedStone.height + analyzedStone.width) > stoneMax) {
                    stone = analyzedStone;
                }
            }
            if(stones.isEmpty())
            {
                telemetry.addLine("No stones detected");
            }
            else
            {
                telemetry.addLine(String.format("Stone: Angle=%s",  stone.angle));
                telemetry.addData("Height", stone.height);
                telemetry.addData("Width", stone.width);
                telemetry.addData("Number of Rings", pipeline.position);
                sleep(50);
                if (stone.height >= 50) {
                    pipeline.position = StoneOrientationExample.StoneOrientationAnalysisPipeline.Rings.FOUR;
                }
                else {
                    pipeline.position = StoneOrientationExample.StoneOrientationAnalysisPipeline.Rings.ONE;
                }

            }


            telemetry.update();
        }

        if (pipeline.position == StoneOrientationExample.StoneOrientationAnalysisPipeline.Rings.FOUR) {
            targetZoneX = 120;
            targetZoneY = -20;
        }
        else if (pipeline.position == StoneOrientationExample.StoneOrientationAnalysisPipeline.Rings.ONE) {
            targetZoneX = 80;
            targetZoneY = -5;
        }
        else {
            targetZoneX = 65;
            targetZoneY = -20;
        }

        Trajectory traj = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineTo(new Vector2d(50, 10) )
                .splineTo(new Vector2d(targetZoneX, targetZoneY), Math.toRadians(0))
                .build();

        drive.followTrajectory(traj);

        //drive.setGrabberPosition(0);

        //drive.moveArmPosition(6100,1,false);

        //drive.moveArmPosition(6000,1,true);

        drive.followTrajectory(
                drive.trajectoryBuilder(traj.end())
                        .lineToLinearHeading(new Pose2d(17, -7, Math.toRadians(180)))
                        .build()
        );

        //drive.moveArmPosition(6100,1,false);

        Trajectory traj2 = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(targetZoneX, targetZoneY, Math.toRadians(0)))
                .build();

        drive.followTrajectory(traj2);


        //drive.moveArmPosition(6100,1,true);

        drive.followTrajectory(
                drive.trajectoryBuilder(traj.end())
                        .lineToLinearHeading(new Pose2d(70, 5, Math.toRadians(180)))
                        .build()
        );

        externalThread.start();

        drive.switchState(threadState.SHOOT);


        while (threadState == threadState.SHOOT && opModeIsActive()) {

        }
        externalThread.interrupt();

        Trajectory traj5 = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(80, 5, Math.toRadians(180)))
                .build();

        drive.followTrajectory(traj5);

        startPose = traj5.end();
        //Autonomous Motion Program Above

        OpStorage.currentPose = drive.getPoseEstimate();

    }
}