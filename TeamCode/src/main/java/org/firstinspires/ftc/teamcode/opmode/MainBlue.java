package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
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


@TeleOp(name="MainBlue", group = "Iterative Opmode")

public class MainBlue extends LinearOpMode {

    private double multiplier = 1;
    private double turnMultiplier = 0.8;

    private boolean toggleGrabber = false;

    private boolean toggleFOD = true;
    private boolean toggleShooter = false;
    private boolean toggleShooterServo = false;

    public static boolean opmodeActive = false;

    private enum Mode {
        NORMAL_CONTROL,
        GO_TO_ORGIN
    }

    private Mode currentMode = Mode.NORMAL_CONTROL;

    private ElapsedTime holdTimerA = new ElapsedTime();
    private ElapsedTime holdTimerB = new ElapsedTime();
    private ElapsedTime holdTimerC = new ElapsedTime();
    private ElapsedTime holdTimerD = new ElapsedTime();
    private ElapsedTime holdTimerE = new ElapsedTime();
    private ElapsedTime holdTimerF = new ElapsedTime();
    private ElapsedTime stopTimer = new ElapsedTime();

    private Vector2d targetPosition = new Vector2d(0, 0);

    private Pose2d startPose = new Pose2d(0, 0, Math.toRadians(90));

    private PIDFController headingController = new PIDFController(SampleMecanumDrive.HEADING_PID);

    private static final String VUFORIA_KEY =
            "AepAbMv/////AAABmRGVNg4fdUxina2B1Jz3k+hUk491O0eNAtpdcDb3Bseqn3GGfDsSpQacuSuLyG4ROahR/y4G8EgGfZBKz55UJ6K941STVdEFyHE1Qv90G5PxTiv+KRsVVnrArOxZOOc9feBI9H/HUr/Y7/7vu50BCinB/9NIjGecgT2gum/FZfbSvbH6dDnGproIVOGslCXzI/FDrGD2YswyTg0x0Y9JhZBpxTk+Vc9hIrP5b2lUTf0a7QypUhKoXFFJm+LjltH5eTU62bTUNqgcjrTpnq7x0nFmAkVWjfcCj/96RVUROjgBgo7o3qyNKVX/87XXlXeTMqzjWgi4M4ZKkxrUKoZvx8riR8/XOuuCLgexTm5NrTLr";

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField  = 36 * mmPerInch;

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

    @Override
    public void runOpMode() throws InterruptedException {

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

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap, toggleFOD);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setPoseEstimate(startPose);
        drive.setGrabberPosition(0);
        drive.setCameraPosition(0.62);
        drive.setShooterPower(0);
        setShooterServoPosition(0.55);
        setShooterAngleServoPosition(0.29);

        Vector2d targetAVector = new Vector2d(0, 0);

        double targetAHeading = Math.toRadians(90);

        Thread externalThread = new SampleMecanumDrive.ExternalDriveThread(hardwareMap,drive);
        Thread armThread = new SampleMecanumDrive.ArmDriveThread();

        targetsUltimateGoal.activate();

        waitForStart();

        externalThread.start();
        armThread.start();

        if (isStopRequested()) {
            externalThread.interrupt();
            armThread.interrupt();
            targetsUltimateGoal.deactivate();
            drive.setIntakePower(0);
            return;
        };

        while (opModeIsActive() && !isStopRequested()) {

            //telemetry.addData("shooterspeed", drive.shooter1.getVelocity()/28);

            // check all the trackable targets to see which one (if any) is visible.
            targetVisible = false;
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                    telemetry.addData("Visible Target", trackable.getName());
                    targetVisible = true;

                    // getUpdatedRobotLocation() will return null if no new information is available since
                    // the last time that call was made, or if the trackable is not currently visible.
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedVuforiaCameraFromTarget();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }
                    break;
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

            Pose2d poseEstimate = drive.getPoseEstimate();

            telemetry.addData("mode", currentMode);

            Vector2d input = new Vector2d(
                    -gamepad1.right_stick_y*multiplier,
                    -gamepad1.right_stick_x*multiplier
            ).rotated(-poseEstimate.getHeading());

            setShooterAngleServoPosition(0.29);

            Pose2d driveDirection = new Pose2d();


                    driveDirection = new Pose2d(
                            input.getX(),
                            input.getY(),
                            -gamepad1.left_stick_x*turnMultiplier
                    );

                    drive.setArmPower(gamepad2.left_trigger - gamepad2.right_trigger);

                    if (gamepad2.dpad_down == true && (holdTimerD.time() > 0.3)) {
                        //drive.setShooterServoPosition(0.95);
                        drive.switchStateArm(armState.FORWARD);
                        holdTimerD.reset();
                    }

                    else if (gamepad2.dpad_up == true && (holdTimerD.time() > 0.3)) {
                        drive.switchStateArm(armState.BACK);
                        holdTimerD.reset();
                    }

                    else if (gamepad2.dpad_right == true && (holdTimerD.time() > 0.3)) {
                        drive.switchStateArm(armState.MID);
                        holdTimerD.reset();
                    }
                    else if (gamepad2.dpad_left == true && (holdTimerD.time() > 0.3)) {
                        drive.switchStateArm(armState.WALL);
                        holdTimerD.reset();
                    }

                    if (gamepad1.back == true && (holdTimerB.time() > 0.8)) {
                        drive.setShooterAngleServoPosition(drive.shooterAngleServo.getPosition() - 0.05);
                        holdTimerB.reset();
                    }

                    else if (gamepad1.start == true && (holdTimerB.time() > 0.8)) {
                        drive.setShooterAngleServoPosition(drive.shooterAngleServo.getPosition() + 0.05);
                        holdTimerB.reset();
                    }

                    if (drive.shooterAngleServo.getPosition() > 1) {
                        drive.setShooterAngleServoPosition(1);
                    }

                    if (drive.shooterAngleServo.getPosition() < 0) {
                        drive.setShooterAngleServoPosition(0);
                    }


                    /*if (gamepad1.b) {
                        Trajectory traj1 = drive.trajectoryBuilder(poseEstimate)
                                .splineTo(targetAVector, targetAHeading)
                                .build();

                        drive.followTrajectoryAsync(traj1);
                    }*/
                    if (gamepad1.y) {
                        multiplier = 0.4;
                        turnMultiplier = 0.4;
                    }

                    if (gamepad1.b) {
                        multiplier = 1;
                        turnMultiplier = 0.8;
                    }

                    if (gamepad1.left_stick_button) {
                        multiplier = -1;
                        turnMultiplier = 0.8;
                    }

                    if(gamepad1.x) {
                        Pose2d poseReset = new Pose2d(poseEstimate.getX(), poseEstimate.getY(), Math.toRadians(0));
                        drive.setPoseEstimate(poseReset);
                    }

                    if(!toggleFOD) {
                        driveDirection = new Pose2d(
                                -gamepad1.right_stick_y*multiplier,
                                -gamepad1.right_stick_x*multiplier,
                                -gamepad1.left_stick_x*turnMultiplier
                        );
                    }



                    if (gamepad1.a == true) {
                        if ((toggleGrabber == true) && (holdTimerA.time() > 0.5)) {
                            drive.setGrabberPosition(1);
                            toggleGrabber = false;
                            holdTimerA.reset();
                        } else if ((toggleGrabber == false) && (holdTimerA.time() > 0.5)) {
                            drive.setGrabberPosition(0);
                            toggleGrabber = true;
                            holdTimerA.reset();
                        }
                    }

                    /*if (gamepad1.right_bumper == true) {
                        if ((toggleShooter == true) && (holdTimerB.time() > 0.5)) {
                            drive.setShooterPower(1);
                            toggleShooter = false;
                            holdTimerB.reset();
                        } else if ((toggleShooter == false) && (holdTimerB.time() > 0.5)) {
                            drive.setShooterPower(0);
                            toggleShooter = true;
                            holdTimerB.reset();
                        }
                    }*/


                    if (gamepad2.y/*left bumper for gp1*/ == true && (holdTimerC.time() > 0.3) && (threadState == threadState.NULL || threadState == threadState.INTAKE) && drive.getShooterPower() != 0) {
                        //drive.setShooterServoPosition(0.95);
                        drive.switchState(threadState.SHOOT);
                        holdTimerC.reset();
                    }

                    /*if (gamepad1.dpad_left == true && (holdTimerC.time() > 0.3) && (threadState == threadState.NULL || threadState == threadState.INTAKE) && drive.getShooterPower() == 0) {
                        //drive.setShooterServoPosition(0.95);
                        Pose2d poseReset = new Pose2d(poseEstimate.getX(), poseEstimate.getY(), Math.toRadians(-90));
                        drive.setPoseEstimate(poseReset);
                        drive.setShooterPower(-0.9);
                        drive.turn(Math.toRadians(170));
                        drive.switchState(threadState.SHOOT_SPIN);
                        holdTimerC.reset();
                    }*/

                    if (gamepad1.dpad_left == true && (holdTimerC.time() > 0.3) && (threadState == threadState.NULL || threadState == threadState.INTAKE) && drive.getShooterPower() == 0) {
                        if (targetVisible) {
                            //drive.setShooterServoPosition(0.95);
                            drive.setShooterPower(-0.9);
                            VectorF translation = lastLocation.getTranslation();
                            double error = Math.toDegrees(Math.asin((translation.get(0) / mmPerInch)/(translation.get(2) / mmPerInch)));
                            Pose2d poseReset = new Pose2d(poseEstimate.getX(), poseEstimate.getY(), Math.toRadians(90 + error));
                            drive.setPoseEstimate(poseReset);
                            drive.turn(Math.toRadians(180 - error),Math.toRadians(360),Math.toRadians(270));
                            while (drive.isBusy()) {}
                            drive.setIntakePower(0);
                            drive.setShooterServoPosition(0.55);
                            drive.setShooterPower(-0.9);
                            drive.setIntakePower(0);
                            Thread.sleep((long) 300);
                            for(int i=0; i<3; i++) {
                                Thread.sleep((long) 150);
                                drive.setShooterServoPosition(1);
                                Thread.sleep((long) 150);
                                drive.setShooterServoPosition(0.55);
                            }
                            sleep((long) 300);
                            drive.setShooterPower(0);
                            drive.switchState(SampleMecanumDrive.ThreadState.NULL);
                            holdTimerC.reset();
                        }
                    }

                    if (gamepad1.dpad_right == true && (holdTimerC.time() > 0.3) && (threadState == threadState.NULL || threadState == threadState.INTAKE) && drive.getShooterPower() == 0) {
                        if (targetVisible) {
                            //drive.setShooterServoPosition(0.95);
                            drive.setShooterAngleServoPosition(0.32);
                            drive.setShooterPower(-0.75);
                            VectorF translation = lastLocation.getTranslation();
                            double error = Math.toDegrees(Math.asin((translation.get(0) / mmPerInch)/(translation.get(2) / mmPerInch)));
                            double angularError = 14 - error;
                            Pose2d poseReset = new Pose2d(poseEstimate.getX(), poseEstimate.getY(), Math.toRadians(90 + error));
                            drive.setPoseEstimate(poseReset);
                            drive.turn(Math.toRadians(180 + angularError));
                            while (drive.isBusy()) {}
                            int angle = 8;
                            for(int i=0; i<3; i++) {
                                sleep((long) 300);
                                drive.setShooterServoPosition(1);
                                sleep((long) 300);
                                drive.setShooterServoPosition(0.5);
                                if (i<2) {
                                    drive.turn(Math.toRadians(angle));
                                    angle = angle;
                                }
                            }
                            sleep((long) 300);
                            drive.setShooterPower(0);
                            drive.switchState(threadState.NULL);
                            drive.setShooterAngleServoPosition(0.29);
                        }
                    }

                    /*if (gamepad1.dpad_down == true && (holdTimerC.time() > 0.3) && (threadState == threadState.NULL || threadState == threadState.INTAKE) && drive.getShooterPower() == 0) {
                        //drive.setShooterServoPosition(0.95);
                        setShooterAngleServoPosition(0.32);
                        drive.setShooterPower(-0.8);
                        sleep((long) 600);
                        drive.setIntakePower(0);
                        drive.setShooterServoPosition(0.5);
                        drive.setIntakePower(0);
                        int angle = 8;
                        for(int i=0; i<3; i++) {
                            sleep((long) 300);
                            drive.setShooterServoPosition(1);
                            sleep((long) 300);
                            drive.setShooterServoPosition(0.5);
                            if (i<2) {
                                drive.turn(Math.toRadians(angle));
                                angle = angle;
                            }
                        }
                        sleep((long) 300);
                        drive.setShooterPower(0);
                        drive.switchState(threadState.NULL);
                    }*/

                    else if (gamepad2.y/*left bumper for gp1*/ == true && (holdTimerC.time() > 0.3) && (threadState == threadState.NULL || threadState == threadState.INTAKE) && drive.getShooterPower() == 0) {
                        //drive.setShooterServoPosition(0.95);
                        drive.setShooterPower(-0.9);
                        holdTimerC.reset();
                    }

                    if (gamepad1.right_bumper == true && (holdTimerC.time() > 0.3) && (threadState == threadState.INTAKE || threadState == threadState.INTAKE_REVERSE)) {
                        drive.switchState(threadState.NULL);
                        holdTimerC.reset();
                    }
                    else if (gamepad1.right_bumper == true && (holdTimerC.time() > 0.3) && (threadState == threadState.NULL || threadState == threadState.INTAKE_REVERSE)) {
                        drive.switchState(threadState.INTAKE);
                        holdTimerC.reset();
                    }

                    if (gamepad1.left_bumper/*right stick button bumper for gp1*/ == true && (holdTimerC.time() > 0.3) && (threadState == threadState.INTAKE || threadState == threadState.NULL)) {
                        drive.switchState(threadState.INTAKE_REVERSE);
                        holdTimerC.reset();
                    }


                telemetry.addData("x", poseEstimate.getX());
                telemetry.addData("y", poseEstimate.getY());
                telemetry.addData("heading", Math.toDegrees(poseEstimate.getHeading()));
                telemetry.addData("arm position", drive.getArmPosition());
                telemetry.addData("arm touch", drive.getArmTouch());
                telemetry.addData("arm power", drive.wobbleArm.getPower());
                telemetry.addData("arm target", drive.getStateArm());
                telemetry.addData("shooter servo", drive.getShooterAngleServoPosition());
                telemetry.update();


                headingController.update(poseEstimate.getHeading());

                drive.setWeightedDrivePower(driveDirection);

                drive.update();
            }

        targetsUltimateGoal.deactivate();

        armThread.interrupt();
        externalThread.interrupt();
    }
}