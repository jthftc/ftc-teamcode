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
import org.firstinspires.ftc.teamcode.vision.UGRectDetector;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import static org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive.armState;
import static org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive.setShooterAngleServoPosition;
import static org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive.setShooterServoPosition;
import static org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive.threadState;


import java.util.ArrayList;

@Autonomous(name = "Auto", group = "Concept")
public class Auto extends LinearOpMode {

    UGRectDetector UGRectDetector;

    ElapsedTime timerA = new ElapsedTime();

    double lengthOfOneMat = 23.583;

    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Thread externalThread = new SampleMecanumDrive.ExternalDriveThread(hardwareMap, drive);
        Thread armThread = new SampleMecanumDrive.ArmDriveThread();

        Pose2d startPose = new Pose2d(-60, -23.5, Math.toRadians(0));

        drive.setPoseEstimate(startPose);

        drive.resetArmPosition();

        drive.setGrabberPosition(0);

        UGRectDetector = new UGRectDetector(hardwareMap, "Cam");
        UGRectDetector.init();

        drive.setCameraPosition(0.5);

        setShooterAngleServoPosition(0.30);

        setShooterServoPosition(0.5 );

        sleep(2000);

        waitForStart();

        if (isStopRequested()) return;

                //Autonomous Motion Program Below

                int targetZoneX = 0;
                int targetZoneY = 0;
                int targetZoneAngle = 0;


                timerA.reset();

                int isSensed = 0;

                while (opModeIsActive() && isSensed == 0 && timerA.time() < 3) {
                    UGRectDetector.Stack stack = UGRectDetector.getStack();
                    switch (stack) {
                        case ZERO:
                            telemetry.addData("[Ring Stack] >>", "ZERO");
                            telemetry.update();
                            isSensed = 1;
                            break;
                        case ONE:
                            telemetry.addData("[Ring Stack] >>", "ONE");
                            telemetry.update();
                            isSensed = 2;
                            break;
                        case FOUR:
                            telemetry.addData("[Ring Stack] >>", "FOUR");
                            telemetry.update();
                            isSensed = 3;
                            break;
                        default:
                            telemetry.addData("[Ring Stack] >>", "NONE");
                            telemetry.update();
                            break;
                    }
                }

                if (isSensed == 3) {
                    targetZoneX = 50;
                    targetZoneY = -52;

                }
                else if (isSensed == 2) {
                    targetZoneX = 30;
                    targetZoneY = -35;
                }
                else {
                    targetZoneX = 4;
                    targetZoneY = -52;
                }

                armThread.start();

                drive.switchStateArm(armState.BACK);
                drive.setGrabberPosition(0);

                Trajectory traj1;
                Trajectory traj2;
                Trajectory traj3;
                Trajectory traj4;
                Trajectory traj5;
                Trajectory traj6;

                traj1 = drive.trajectoryBuilder(startPose)
                        .lineTo(new Vector2d(-16,-23.5))
                        .build();

                drive.followTrajectory(traj1);

                drive.switchStateArm(armState.WALL);

                //drive.turn(Math.toRadians(targetZoneAngle));

                traj2 = drive.trajectoryBuilder(new Pose2d(traj1.end().getX(),traj1.end().getY(), Math.toRadians(0)))
                        .lineTo(new Vector2d(targetZoneX,targetZoneY))
                        .build();

                drive.followTrajectory(traj2);

                drive.switchStateArm(armState.FORWARD);
                sleep(500);
                drive.setGrabberPosition(1);
                sleep(500);
                drive.switchStateArm(armState.BACK);

                drive.turn(Math.toRadians(180));

                drive.switchStateArm(armState.FORWARD);

                drive.setPoseEstimate(new Pose2d(targetZoneX,targetZoneY,Math.toRadians(180)));

                traj3 = drive.trajectoryBuilder(drive.getPoseEstimate())
                            .lineTo(new Vector2d(-35, -52))
                            .build();

                drive.followTrajectory(traj3);

                drive.setGrabberPosition(0);
                sleep(500);
                drive.switchStateArm(armState.WALL);


                traj4 = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .lineTo(new Vector2d(targetZoneX, targetZoneY))
                        .build();

                drive.followTrajectory(traj4);

                drive.turn(Math.toRadians(90));

                drive.switchStateArm(armState.FORWARD);
                sleep(500);
                drive.setGrabberPosition(1);
                sleep(500);
                drive.switchStateArm(armState.BACK);

                drive.setPoseEstimate(new Pose2d(targetZoneX,targetZoneY,Math.toRadians(-90)));


                traj5 = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineTo(new Vector2d(-5, -45))
                .build();

                drive.followTrajectory(traj5);

                drive.turn(Math.toRadians(-90));

                drive.setPoseEstimate(new Pose2d(-5,-45,Math.toRadians(180)));






                externalThread.start();

                drive.switchState(threadState.SHOOT_VELO);

                while (threadState == threadState.SHOOT_VELO && opModeIsActive()) {

                }

                externalThread.interrupt();

                traj6 = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .lineTo(new Vector2d(10, -45))
                        .build();

                drive.followTrajectory(traj6);


                armThread.interrupt();

        //Autonomous Motion Program Above

        OpStorage.currentPose = drive.getPoseEstimate();

    }
}