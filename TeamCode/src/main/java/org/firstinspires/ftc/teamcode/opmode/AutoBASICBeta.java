package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.storage.OpStorage;
import org.firstinspires.ftc.teamcode.vision.UGRectDetector;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XZY;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive.armState;
import static org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive.setShooterAngleServoPosition;
import static org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive.setShooterServoPosition;
import static org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive.threadState;

@Autonomous(name = "AutoBASICBeta", group = "Concept")
public class AutoBASICBeta extends LinearOpMode {

    private static final String VUFORIA_KEY =
            "AepAbMv/////AAABmRGVNg4fdUxina2B1Jz3k+hUk491O0eNAtpdcDb3Bseqn3GGfDsSpQacuSuLyG4ROahR/y4G8EgGfZBKz55UJ6K941STVdEFyHE1Qv90G5PxTiv+KRsVVnrArOxZOOc9feBI9H/HUr/Y7/7vu50BCinB/9NIjGecgT2gum/FZfbSvbH6dDnGproIVOGslCXzI/FDrGD2YswyTg0x0Y9JhZBpxTk+Vc9hIrP5b2lUTf0a7QypUhKoXFFJm+LjltH5eTU62bTUNqgcjrTpnq7x0nFmAkVWjfcCj/96RVUROjgBgo7o3qyNKVX/87XXlXeTMqzjWgi4M4ZKkxrUKoZvx8riR8/XOuuCLgexTm5NrTLr";

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField  = 36 * mmPerInch;

    private ElapsedTime holdTimerC = new ElapsedTime();


    // Class Members
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;

    /**
     * This is the webcam we are to use. As with other hardware devices such as motors and
     * servos, this device is identified using the robot configuration tool in the FTC application.
     */
    WebcamName webcamName = null;

    private boolean targetVisible = false;
    private float phoneXRotate    = 0;
    private float phoneYRotate    = 0;
    private float phoneZRotate    = 0;

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

        webcamName = hardwareMap.get(WebcamName.class, "Cam");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        parameters.cameraName = webcamName;

        // Make sure extended tracking is disabled for this example.
        parameters.useExtendedTracking = false;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        VuforiaTrackables targetsUltimateGoal = this.vuforia.loadTrackablesFromAsset("UltimateGoal");
        VuforiaTrackable blueTowerGoalTarget = targetsUltimateGoal.get(0);
        blueTowerGoalTarget.setName("Blue Tower Goal Target");
        VuforiaTrackable redTowerGoalTarget = targetsUltimateGoal.get(1);
        redTowerGoalTarget.setName("Red Tower Goal Target");

        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsUltimateGoal);

        blueTowerGoalTarget.setLocation(OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));
        redTowerGoalTarget.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        final float CAMERA_FORWARD_DISPLACEMENT  = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot-center
        final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix cameraLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XZY, DEGREES, 90, 90, 0));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setCameraLocationOnRobot(parameters.cameraName, cameraLocationOnRobot);
        }

        drive.setCameraPosition(0.62);

        setShooterAngleServoPosition(0.30);

        setShooterServoPosition(0.5 );

        sleep(2000);

        targetsUltimateGoal.activate();


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


                externalThread.start();

                targetVisible = false;
                while (targetVisible != true) {
                    for (VuforiaTrackable trackable : allTrackables) {
                        if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                            telemetry.addData("Visible Target", trackable.getName());
                            targetVisible = true;

                            // getUpdatedRobotLocation() will return null if no new information is available since
                            // the last time that call was made, or if the trackable is not currently visible.
                            OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedVuforiaCameraFromTarget();
                            if (robotLocationTransform != null) {
                                lastLocation = robotLocationTransform;
                            }
                            break;
                        }
                    }
                }

                // Provide feedback as to where the robot is located (if we know).
                if (targetVisible) {
                    // express position (translation) of robot in inches.
                    VectorF translation = lastLocation.getTranslation();

                    telemetry.addData("Loc (px", "{X, Y, Z} = %.1f, %.1f, %.1f",
                            lastLocation.toVector().get(0) / mmPerInch, lastLocation.toVector().get(1) / mmPerInch, lastLocation.toVector().get(2) / mmPerInch);


                    telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                            translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

                    telemetry.addData("Angle off", Math.toDegrees(Math.asin((translation.get(0) / mmPerInch)/(translation.get(2) / mmPerInch))));

                    telemetry.addData("Angle off of Candle", 16 - Math.toDegrees(Math.asin((translation.get(0) / mmPerInch)/(translation.get(2) / mmPerInch))));

                    // express the rotation of the robot in degrees.
                    Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                    telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
                }
                else {
                    telemetry.addData("Visible Target", "none");
                }
                telemetry.update();

                double error = 0;

                if (targetVisible) {
                    //drive.setShooterServoPosition(0.95);
                    drive.setShooterPower(-0.9);
                    VectorF translation = lastLocation.getTranslation();
                    error = Math.toDegrees(Math.asin((translation.get(0) / mmPerInch)/(translation.get(2) / mmPerInch)));
                    Pose2d poseReset = new Pose2d(-5, -23.5, Math.toRadians(-90 + error));
                    drive.setPoseEstimate(poseReset);
                    drive.turn(Math.toRadians(180 - error),Math.toRadians(360),Math.toRadians(270));
                    while (drive.isBusy()) {}
                    drive.setIntakePower(0);
                    drive.setShooterServoPosition(0.55);
                    drive.setShooterPower(-0.9);
                    drive.setIntakePower(0);
                    Thread.sleep((long) 300);
                }

                drive.switchState(threadState.SHOOT_VELO);

                while (threadState == threadState.SHOOT_VELO && opModeIsActive()) {

                }

                externalThread.interrupt();

                drive.turn(Math.toRadians(-1*(180 - error)));

                traj2 = drive.trajectoryBuilder(new Pose2d(traj1.end().getX(),traj1.end().getY(), Math.toRadians(0)))
                        .lineTo(new Vector2d(15,-23.5))
                        .build();

                drive.followTrajectory(traj2);

                armThread.interrupt();

        //Autonomous Motion Program Above

        OpStorage.currentPose = drive.getPoseEstimate();

        targetsUltimateGoal.deactivate();

    }
}