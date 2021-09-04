package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.storage.OpStorage;
import org.firstinspires.ftc.teamcode.vision.UGRectDetector;

import static org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive.armState;
import static org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive.setShooterAngleServoPosition;
import static org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive.setShooterServoPosition;
import static org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive.threadState;

@Autonomous(name = "AutoBASICBlue", group = "Concept")
public class AutoBASICBlue extends LinearOpMode {

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
                        .lineTo(new Vector2d(-5,-23.5))
                        .build();

                drive.followTrajectory(traj1);

                drive.turn(Math.toRadians(200));

                externalThread.start();

                drive.switchState(threadState.SHOOT_VELO);

                while (threadState == threadState.SHOOT_VELO && opModeIsActive()) {

                }

                externalThread.interrupt();

                drive.turn(Math.toRadians(-200));

                traj2 = drive.trajectoryBuilder(new Pose2d(traj1.end().getX(),traj1.end().getY(), Math.toRadians(0)))
                        .lineTo(new Vector2d(15,-23.5))
                        .build();

                drive.followTrajectory(traj2);

                armThread.interrupt();

        //Autonomous Motion Program Above

        OpStorage.currentPose = drive.getPoseEstimate();

    }
}