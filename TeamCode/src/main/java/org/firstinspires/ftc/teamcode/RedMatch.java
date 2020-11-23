/* Copyright (c) 2017 FIRST. All rights reserved.
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
 * Copyright (c) 2017 FIRST. All rights reserved.
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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;


/**
 * This file illustrates the concept of driving a path based on Gyro heading and encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that you have a Modern Robotics I2C gyro with the name "gyro"
 *   otherwise you would use: PushbotAutoDriveByEncoder;
 *
 *  This code requires that the drive Motors have been configured such that a positive
 *  power command moves them forward, and causes the encoders to count UP.
 *
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 *  In order to calibrate the Gyro correctly, the robot must remain stationary during calibration.
 *  This is performed when the INIT button is pressed on the Driver Station.
 *  This code assumes that the robot is stationary when the INIT button is pressed.
 *  If this is not the case, then the INIT should be performed again.
 *
 *  Note: in this example, all angles are referenced to the initial coordinate frame set during the
 *  the Gyro Calibration process, or whenever the program issues a resetZAxisIntegrator() call on the Gyro.
 *
 *  The angle of movement/rotation is assumed to be a standardized rotation around the robot Z axis,
 *  which means that a Positive rotation is Counter Clock Wise, looking down on the field.
 *  This is consistent with the FTC field coordinate conventions set out in the document:
 *  ftc_app\doc\tutorial\FTC_FieldCoordinateSystemDefinition.pdf
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name = "RedMatch", group = "Concept")

public class RedMatch extends LinearOpMode {

    /* Declare OpMode members. */   // Use a Pushbot's hardware
    BNO055IMU gyro;                 // Additional Gyro device
    Orientation angles = new Orientation();
    public DcMotor front_left  = null;
    public DcMotor front_right = null;
    public DcMotor back_left   = null;
    public DcMotor back_right  = null;
    public DcMotor left_lift   = null;
    public DcMotor right_lift  = null;
    private Servo gripper1 = null;
    private Servo gripper2 = null;
    private Servo left_intake = null;
    private Servo right_intake = null;
    private Servo pullArm = null;
    private Servo grabber = null;
    private Servo adjustArm = null;
    private double lastAngle = 0;
    private int skyloc = 0;
    private int addon = 0;

    ElapsedTime grabTime = new ElapsedTime();
    ElapsedTime extendTime = new ElapsedTime();
    ElapsedTime adjustTime = new ElapsedTime();
    ElapsedTime latchTime = new ElapsedTime();
    ElapsedTime liftTime = new ElapsedTime();
    ElapsedTime tankTime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 560 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.14592);

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double     DRIVE_SPEED             = 0.2;     // Nominal speed for better accuracy.
    static final double     TURN_SPEED              = 0.3;     // Nominal half speed for better accuracy.

    static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.05;     // Larger is more responsive, but also less stable

    //OpenCV Variables
    private static int valMid = -1;
    private static int valLeft = -1;
    private static int valRight = -1;
    private static int valLeftCheck = -1;
    private static int valRightCheck = -1;

    private static float rectHeight = .6f/8f;
    private static float rectWidth = 1.5f/8f;

    private static float offsetX = 0f/8f;//changing this moves the three rects and the three circles left or right, range : (-2, 2) not inclusive
    private static float offsetY = 0f/8f;//changing this moves the three rects and circles up or down, range: (-4, 4) not inclusive

    private static float[] midPos = {4f/8f+offsetX, 2.5f/8f+offsetY};//0 = col, 1 = row
    private static float[] leftPos = {1f/8f+offsetX, 2.5f/8f+offsetY};
    private static float[] rightPos = {7f/8f+offsetX, 2.5f/8f+offsetY};
    private static float[] leftCheck = {6f/8f+offsetX, 2.5f/8f+offsetY};
    private static float[] rightCheck = {7.4f/8f+offsetX, 2.5f/8f+offsetY};

    //moves all rectangles right or left by amount. units are in ratio to monitor

    private final int rows = 640;
    private final int cols = 480;
    WebcamName Cam = null;
    OpenCvCamera phoneCam;
    @Override
    public void runOpMode() {
        //OpenCv Init
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        Cam = hardwareMap.get(WebcamName.class, "Cam");
        //P.S. if you're using the latest version of easyopencv, you might need to change the next line to the following:
        phoneCam = OpenCvCameraFactory.getInstance().createWebcam(Cam, cameraMonitorViewId);
        //phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);//remove this

        phoneCam.openCameraDevice();//open camera
        phoneCam.setPipeline(new RedMatch.StageSwitchingPipeline());//different stages
        phoneCam.startStreaming(rows, cols, OpenCvCameraRotation.UPSIDE_DOWN);//display on RC
        //width, height
        //width = height in this case, because camera is in portrait mode.
        //Gyro Init
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        telemetry.addData("Last Angle:", lastAngle);
        gyro = hardwareMap.get(BNO055IMU.class, "imu");
        /*
         * Initialize the standard drive system variables.
         * The init() method of the hardware class does most of the work here
         */
        front_left      = hardwareMap.get(DcMotor.class, "front_left");
        front_right     = hardwareMap.get(DcMotor.class, "front_right");
        back_left       = hardwareMap.get(DcMotor.class, "back_left");
        back_right      = hardwareMap.get(DcMotor.class, "back_right");
        left_lift       = hardwareMap.get(DcMotor.class, "left_lift");
        right_lift      = hardwareMap.get(DcMotor.class, "right_lift");
        gripper1        = hardwareMap.get(Servo.class, "gripper1");
        gripper2        = hardwareMap.get(Servo.class, "gripper2");
        grabber        = hardwareMap.get(Servo.class, "grabber");
        left_intake        = hardwareMap.get(Servo.class, "left_intake");
        right_intake        = hardwareMap.get(Servo.class, "right_intake");
        pullArm       = hardwareMap.get(Servo.class, "pullArm");
        adjustArm       = hardwareMap.get(Servo.class, "adjustArm");


        // Ensure the robot it stationary, then reset the encoders and calibrate the gyro.
        front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Send telemetry message to alert driver that we are calibrating;
        telemetry.addData(">", "Calibrating Gyro");    //
        telemetry.update();

        gyro.initialize(parameters);

        // make sure the gyro is calibrated before continuing
        while (!isStopRequested() && gyro.isGyroCalibrated())  {
            sleep(50);
            idle();
        }

        telemetry.addData(">", "Robot Ready.");    //
        telemetry.update();

        front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        grabber.setPosition(0.5);
        left_intake.setPosition(0.4);
        right_intake.setPosition(0.5);
        adjustArm.setPosition(0.25);
        gripper1.setPosition(0);
        pullArm.setPosition(0.84);
        gripper2.setPosition(1);
        // Wait for the game to start (Display Gyro value), and reset gyro before we move..
        while (!isStarted()) {
            angles   = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("Robot Heading = ", angles.firstAngle);
            telemetry.update();
            telemetry.addData("Values", valLeft+" "+valMid+" "+valRight+" "+valLeftCheck+" "+valRightCheck);
            telemetry.addData("Height", rows);
            telemetry.addData("Width", cols);
        }

        while (opModeIsActive() && skyloc==0) {
            telemetry.addData("Values", valLeft+" "+valMid+" "+valRight+" "+valLeftCheck+" "+valRightCheck);
            telemetry.addData("Height", rows);
            telemetry.addData("Width", cols);
            if (valLeft < 100 && valMid > 100 && valRight > 100) {
                telemetry.addData("SkyStone Position", "Left");
                skyloc = 1;
            }
            else if (valLeft > 100 && valMid < 100 && valRight > 100) {
                telemetry.addData("SkyStone Position", "Middle");
                skyloc = 2;
            }
            else if (valLeft > 100 && valMid > 100 && valRight < 100) {
                telemetry.addData("SkyStone Position", "Right");
                skyloc = 3;
            }
            telemetry.update();
            sleep(100);
        }
        if (skyloc != 3) {
            strafeUntil(0.2, 0);
            if (skyloc == 2) {
                addon = 16;
            }
            else {
                addon = 8;
            }
        }
        holdDegPlusArm(0, 0.3, 1, true);
        driveInch(28, 0.9, 0);
        grab(true);
        driveInch(-4, 0.9);
        turnDeg(-90, 0.7);
        driveInch(80+addon, 0.9, -90);
        lift(30, 1);
        turnDeg(0, 0.5);
        driveInch(15, 0.9, 0);
        latch(true);
        driveInch(-40, 0.9, 0);
        lift(-27, 1);
        grab(false);
        latch(false);
        lift(-3, 1);

        /*strafe(-30,0.4);
        sleep(500);
        driveInch(70, 0.9, lastAngle);
        //holdDeg(0,0.3,1);
        sleep(500);
        strafe(40, 0.4);*/
        //holdDeg(0,0.3,1);
        //turnDeg(45, 0.4);
        //sleep(10000);*/
        /*extend(3,1);
        grab(true);
        adjust(1);
        driveInch(-10, 0.2);
        turnDeg(-90,0.2);
        driveInch(60, 0.8);
        turnDeg(0,0.2);
        scissorset(0.8,1);
        driveInch(5, 0.6);
        latch(true);
        scissorset(-0.8,1);
        grab(false);
        driveInch(-50, 0.4);
        stop();*/
        /*turnDeg(-90, 0.3);
        driveInch(-10, 0.4);
        latch(true);
        sleep(2000);
        driveInch(10, 0.4);
        turnDeg(-135, 0.3);
        driveInch(-50, 0.4);
        latch(false);
        sleep(2000);
        driveInch(10, 0.4);
        turnDeg(-45, 0.3);2
        driveInch(19, 0.4);
        turnDeg(-90, 0.3);
        driveInch(8, 0.4);*/

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        // Put a hold after each turn
        // Drive REV 48 inches

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }


    /**
     *  Method to drive on a fixed compass bearing (angle), based on encoder counts.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the desired position
     *  2) Driver stops the opmode running.
     *
     \     Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
   /*public void strafe(double time, double speed) {
       grabTime.reset();
           while (grabTime.time() < time) {
               front_left.setPower(-speed);
               front_right.setPower(speed);
               back_left.setPower(speed);
               back_right.setPower(-speed);
           }
   }*/
    public void grab(boolean val) {
        grabTime.reset();
        if (val == true) {
            while (grabTime.time() < 0.5) {
                grabber.setPosition(1);
            }
            while (grabTime.time() < 1.5) {
                adjustArm.setPosition(0.6);
            }
        }
        else {
            while (grabTime.time() < 0.5) {
                grabber.setPosition(0.5);
            }
        }
    }
    public void adjust(double val) {
        adjustTime.reset();
        while (adjustTime.time() < 2) {
            adjustArm.setPosition(val);
        }
    }
    public void latch(boolean val) {

        latchTime.reset();
        if (val == true) {
            while (latchTime.time() < 0.5) {
                gripper1.setPosition(1);
                gripper2.setPosition(0);
            }

        }
        else {
            while (latchTime.time() < 1) {
                gripper1.setPosition(0);
                gripper2.setPosition(1);
            }
        }
    }
    public void lift (double distance, double speed) {

        int     newLeftLiftTarget;
        int     newRightLiftTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;


        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * COUNTS_PER_INCH);

            newLeftLiftTarget = left_lift.getCurrentPosition() + moveCounts;
            newRightLiftTarget = right_lift.getCurrentPosition() + moveCounts;


            // Set Target and Turn On RUN_TO_POSITION
            left_lift.setTargetPosition(newLeftLiftTarget);
            right_lift.setTargetPosition(newRightLiftTarget);

            left_lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            right_lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            // start motion.


            // keep looping while we are still active, and BOTH motors are running.
            if (distance > 0) {
                while (left_lift.getCurrentPosition() < newLeftLiftTarget && opModeIsActive()) {
                    // adjust relative speed based on heading error.
                    left_lift.setPower(speed);
                    right_lift.setPower(speed);

                }
            }
            else if (distance < 0) {
                while (left_lift.getCurrentPosition() > newLeftLiftTarget && opModeIsActive()) {
                    // adjust relative speed based on heading error.
                    left_lift.setPower(-speed);
                    right_lift.setPower(-speed);

                }
            }
            left_lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            right_lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            left_lift.setPower(0);
            right_lift.setPower(0);

            // Turn off RUN_TO_POSITION

        }
    }
    public void driveInch (double distance, double speed) {

        int     newFrontLeftTarget;
        int     newFrontRightTarget;
        int     newBackLeftTarget;
        int     newBackRightTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  newspeed = (speed/3) *2;
        double  thirdspeed = (speed/3);
        double  orispeed = speed;
        double  leftSpeed;
        double  rightSpeed;
        double customSpeed = speed;
        Orientation angles   = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double angle = angles.firstAngle;
        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * COUNTS_PER_INCH);
            if (distance < 0 ) {
                customSpeed = -customSpeed;
            }
            newFrontLeftTarget = front_left.getCurrentPosition() - moveCounts;
            newFrontRightTarget = front_right.getCurrentPosition() + moveCounts;
            newBackLeftTarget = back_left.getCurrentPosition() - moveCounts;
            newBackRightTarget = back_right.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            front_left.setTargetPosition(newFrontLeftTarget);
            front_right.setTargetPosition(newFrontRightTarget);
            back_left.setTargetPosition(newBackLeftTarget);
            back_right.setTargetPosition(newBackRightTarget);

            front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            front_left.setPower(speed);
            front_right.setPower(speed);
            back_left.setPower(speed);
            back_right.setPower(speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (front_left.isBusy() && back_left.isBusy() && back_right.isBusy() && opModeIsActive()) {
                // adjust relative speed based on heading error.

                if (front_left.getCurrentPosition() - 1000 < newFrontLeftTarget) {
                    double adjustspeed = newspeed*((newFrontLeftTarget-front_left.getCurrentPosition())/1000);
                    speed = adjustspeed + thirdspeed;
                }
                else {
                    speed = orispeed;
                }
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                front_left.setPower(leftSpeed);
                front_right.setPower(rightSpeed);
                back_left.setPower(leftSpeed);
                back_right.setPower(rightSpeed);
            }
            front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            tankTime.reset();
            while(tankTime.time() < 0.2) {
                if (distance > 0) {
                    front_left.setPower(-speed);
                    front_right.setPower(speed);
                    back_left.setPower(-speed);
                    back_right.setPower(speed);
                }
                else {
                    front_left.setPower(speed);
                    front_right.setPower(-speed);
                    back_left.setPower(speed);
                    back_right.setPower(-speed);
                }
            }
            // Stop all motion;
            front_left.setPower(0);
            front_right.setPower(0);
            back_left.setPower(0);
            back_right.setPower(0);

            // Turn off RUN_TO_POSITION

        }
    }
    public void driveInch (double distance, double speed, double setangle) {

        int     newFrontLeftTarget;
        int     newFrontRightTarget;
        int     newBackLeftTarget;
        int     newBackRightTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  newspeed = (speed/3) *2;
        double  thirdspeed = (speed/3);
        double  orispeed = speed;
        double  leftSpeed;
        double  rightSpeed;
        double customSpeed = speed;
        Orientation angles   = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double angle = angles.firstAngle;
        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * COUNTS_PER_INCH);
            if (distance < 0 ) {
                customSpeed = -customSpeed;
            }
            newFrontLeftTarget = front_left.getCurrentPosition() - moveCounts;
            newFrontRightTarget = front_right.getCurrentPosition() + moveCounts;
            newBackLeftTarget = back_left.getCurrentPosition() - moveCounts;
            newBackRightTarget = back_right.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            front_left.setTargetPosition(newFrontLeftTarget);
            front_right.setTargetPosition(newFrontRightTarget);
            back_left.setTargetPosition(newBackLeftTarget);
            back_right.setTargetPosition(newBackRightTarget);

            front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            front_left.setPower(speed);
            front_right.setPower(speed);
            back_left.setPower(speed);
            back_right.setPower(speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (front_left.isBusy() && back_left.isBusy() && back_right.isBusy() && opModeIsActive()) {
                // adjust relative speed based on heading error.

                if (front_left.getCurrentPosition() - 1000 < newFrontLeftTarget) {
                    double adjustspeed = newspeed*((newFrontLeftTarget-front_left.getCurrentPosition())/1000);
                    speed = adjustspeed + thirdspeed;
                }
                else {
                    speed = orispeed;
                }
                error = getError(setangle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                front_left.setPower(leftSpeed);
                front_right.setPower(rightSpeed);
                back_left.setPower(leftSpeed);
                back_right.setPower(rightSpeed);
            }
            front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            tankTime.reset();
            while(tankTime.time() < 0.2) {
                if (distance > 0) {
                    front_left.setPower(-speed);
                    front_right.setPower(speed);
                    back_left.setPower(-speed);
                    back_right.setPower(speed);
                }
                else {
                    front_left.setPower(speed);
                    front_right.setPower(-speed);
                    back_left.setPower(speed);
                    back_right.setPower(-speed);
                }
            }
            // Stop all motion;
            front_left.setPower(0);
            front_right.setPower(0);
            back_left.setPower(0);
            back_right.setPower(0);

            // Turn off RUN_TO_POSITION

        }
    }
    public void strafe (double distance, double speed, int keepangle) {

        int     newFrontLeftTarget;
        int     newFrontRightTarget;
        int     newBackLeftTarget;
        int     newBackRightTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  newspeed = (speed/3) *2;
        double  thirdspeed = (speed/3);
        double  orispeed = speed;
        double  leftSpeed;
        double  rightSpeed;
        double customSpeed = speed;
        Orientation angles   = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double angle = angles.firstAngle;
        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * COUNTS_PER_INCH);
            if (distance > 0 ) {
                customSpeed = -customSpeed;
            }
            newFrontLeftTarget = front_left.getCurrentPosition() - moveCounts;
            newFrontRightTarget = front_right.getCurrentPosition() - moveCounts;
            newBackLeftTarget = back_left.getCurrentPosition() + moveCounts;
            newBackRightTarget = back_right.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            front_left.setTargetPosition(newFrontLeftTarget);
            front_right.setTargetPosition(newFrontRightTarget);
            back_left.setTargetPosition(newBackLeftTarget);
            back_right.setTargetPosition(newBackRightTarget);

            front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            front_left.setPower(speed);
            front_right.setPower(speed);
            back_left.setPower(speed);
            back_right.setPower(speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (front_left.isBusy() && back_left.isBusy() && back_right.isBusy() && opModeIsActive()) {
                // adjust relative speed based on heading error.


                error = getError(keepangle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                /*if (distance < 0)
                    steer *= -1.0;*/

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                front_left.setPower(leftSpeed);
                front_right.setPower(rightSpeed);
                back_left.setPower(leftSpeed);
                back_right.setPower(rightSpeed);
            }
            front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // Stop all motion;
            front_left.setPower(0);
            front_right.setPower(0);
            back_left.setPower(0);
            back_right.setPower(0);

            // Turn off RUN_TO_POSITION

        }
    }
    public void strafeUntil (double speed, double keepangle) {

        int     newFrontLeftTarget;
        int     newFrontRightTarget;
        int     newBackLeftTarget;
        int     newBackRightTarget;
        int     moveCounts;
        int     distance = -1000;
        double  max;
        double  error;
        double  steer;
        double  newspeed = (speed/3) *2;
        double  thirdspeed = (speed/3);
        double  orispeed = speed;
        double  leftSpeed;
        double  rightSpeed;
        double customSpeed = speed;
        Orientation angles   = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double angle = angles.firstAngle;
        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * COUNTS_PER_INCH);
            newFrontLeftTarget = front_left.getCurrentPosition() - moveCounts;
            newFrontRightTarget = front_right.getCurrentPosition() - moveCounts;
            newBackLeftTarget = back_left.getCurrentPosition() + moveCounts;
            newBackRightTarget = back_right.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            front_left.setTargetPosition(newFrontLeftTarget);
            front_right.setTargetPosition(newFrontRightTarget);
            back_left.setTargetPosition(newBackLeftTarget);
            back_right.setTargetPosition(newBackRightTarget);

            front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            front_left.setPower(speed);
            front_right.setPower(speed);
            back_left.setPower(speed);
            back_right.setPower(speed);

            // keep looping while we are still active, and BOTH motors are running.
            while ((valLeftCheck + valRightCheck) != 0 && opModeIsActive()) {
                // adjust relative speed based on heading error.
                error = getError(keepangle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                /*if (distance < 0)
                    steer *= -1.0;*/

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                front_left.setPower(leftSpeed);
                front_right.setPower(rightSpeed);
                back_left.setPower(leftSpeed);
                back_right.setPower(rightSpeed);

            }
            front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // Stop all motion;
            front_left.setPower(0);
            front_right.setPower(0);
            back_left.setPower(0);
            back_right.setPower(0);

            // Turn off RUN_TO_POSITION

        }
    }
    /**
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void turnDeg (  double angle, double speed) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
        lastAngle = getTrueAngle(angle);
        telemetry.addData("Last Angle:", lastAngle);
    }

    /**
     *  Method to obtain & hold a heading for a finite amount of time
     *  Move will stop once the requested time has elapsed
     *
     * @param speed      Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */
    public void holdDeg( double angle, double speed, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            telemetry.update();
        }
        lastAngle = getTrueAngle(angle);
        telemetry.addData("Last Angle:", lastAngle);
        // Stop all motion;
        front_left.setPower(0);
        front_right.setPower(0);
        back_left.setPower(0);
        back_right.setPower(0);
    }
    public void holdDegPlusArm( double angle, double speed, double holdTime, boolean arm) {

        if (arm == true) {
            pullArm.setPosition(0.903);
        }
        else {
            pullArm.setPosition(0.84);
        }
        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            telemetry.update();
        }
        lastAngle = getTrueAngle(angle);
        telemetry.addData("Last Angle:", lastAngle);
        // Stop all motion;
        front_left.setPower(0);
        front_right.setPower(0);
        back_left.setPower(0);
        back_right.setPower(0);
    }
    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param PCoeff    Proportional Gain coefficient
     * @return
     */
    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = rightSpeed;
        }

        // Send desired speeds to motors.
        front_left.setPower(leftSpeed);
        front_right.setPower(rightSpeed);
        back_left.setPower(leftSpeed);
        back_right.setPower(rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        angles   = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        robotError = targetAngle - angles.firstAngle;
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    public double getTrueAngle(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle;
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }
    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }
    //OpenCV Begins
    static class StageSwitchingPipeline extends OpenCvPipeline
    {
        Mat yCbCrChan2Mat = new Mat();
        Mat thresholdMat = new Mat();
        Mat all = new Mat();
        List<MatOfPoint> contoursList = new ArrayList<>();

        enum Stage
        {//color difference. greyscale
            detection,//includes outlines
            THRESHOLD,//b&w
            RAW_IMAGE,//displays raw view
        }

        private RedMatch.StageSwitchingPipeline.Stage stageToRenderToViewport = RedMatch.StageSwitchingPipeline.Stage.detection;
        private RedMatch.StageSwitchingPipeline.Stage[] stages = RedMatch.StageSwitchingPipeline.Stage.values();

        @Override
        public void onViewportTapped()
        {
            /*
             * Note that this method is invoked from the UI thread
             * so whatever we do here, we must do quickly.
             */

            int currentStageNum = stageToRenderToViewport.ordinal();

            int nextStageNum = currentStageNum + 1;

            if(nextStageNum >= stages.length)
            {
                nextStageNum = 0;
            }

            stageToRenderToViewport = stages[nextStageNum];
        }

        @Override
        public Mat processFrame(Mat input)
        {
            contoursList.clear();
            /*
             * This pipeline finds the contours of yellow blobs such as the Gold Mineral
             * from the Rover Ruckus game.
             */

            //color diff cb.
            //lower cb = more blue = skystone = white
            //higher cb = less blue = yellow stone = grey
            Imgproc.cvtColor(input, yCbCrChan2Mat, Imgproc.COLOR_RGB2YCrCb);//converts rgb to ycrcb
            Core.extractChannel(yCbCrChan2Mat, yCbCrChan2Mat, 2);//takes cb difference and stores

            //b&w
            Imgproc.threshold(yCbCrChan2Mat, thresholdMat, 102, 255, Imgproc.THRESH_BINARY_INV);

            //outline/contour
            Imgproc.findContours(thresholdMat, contoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
            yCbCrChan2Mat.copyTo(all);//copies mat object
            Imgproc.drawContours(all, contoursList, -1, new Scalar(255, 0, 0), 3, 8);//draws blue contours


            //get values from frame
            double[] pixMid = thresholdMat.get((int)(input.rows()* midPos[1]), (int)(input.cols()* midPos[0]));//gets value at circle
            valMid = (int)pixMid[0];

            double[] pixLeft = thresholdMat.get((int)(input.rows()* leftPos[1]), (int)(input.cols()* leftPos[0]));//gets value at circle
            valLeft = (int)pixLeft[0];

            double[] pixRight = thresholdMat.get((int)(input.rows()* rightPos[1]), (int)(input.cols()* rightPos[0]));//gets value at circle
            valRight = (int)pixRight[0];

            double[] pixLeftCheck = thresholdMat.get((int)(input.rows()* leftCheck[1]), (int)(input.cols()* leftCheck[0]));//gets value at circle
            valLeftCheck = (int)pixLeftCheck[0];

            double[] pixRightCheck = thresholdMat.get((int)(input.rows()* rightCheck[1]), (int)(input.cols()* rightCheck[0]));//gets value at circle
            valRightCheck = (int)pixRightCheck[0];
            //create three points
            Point pointMid = new Point((int)(input.cols()* midPos[0]), (int)(input.rows()* midPos[1]));
            Point pointLeft = new Point((int)(input.cols()* leftPos[0]), (int)(input.rows()* leftPos[1]));
            Point pointRight = new Point((int)(input.cols()* rightPos[0]), (int)(input.rows()* rightPos[1]));
            Point pointLeftCheck = new Point((int)(input.cols()* leftCheck[0]), (int)(input.rows()* leftCheck[1]));
            Point pointRightCheck = new Point((int)(input.cols()* rightCheck[0]), (int)(input.rows()* rightCheck[1]));

            //draw circles on those points
            Imgproc.circle(all, pointMid,5, new Scalar( 255, 0, 0 ),1 );//draws circle
            Imgproc.circle(all, pointLeft,5, new Scalar( 255, 0, 0 ),1 );//draws circle
            Imgproc.circle(all, pointRight,5, new Scalar( 255, 0, 0 ),1 );//draws circle
            Imgproc.circle(all, pointLeftCheck,5, new Scalar( 255, 0, 0 ),1 );//draws circle
            Imgproc.circle(all, pointRightCheck,5, new Scalar( 255, 0, 0 ),1 );//draws circle

            //draw 3 rectangles
            Imgproc.rectangle(//1-3
                    all,
                    new Point(
                            input.cols()*(leftPos[0]-rectWidth/2),
                            input.rows()*(leftPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(leftPos[0]+rectWidth/2),
                            input.rows()*(leftPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(//3-5
                    all,
                    new Point(
                            input.cols()*(midPos[0]-rectWidth/2),
                            input.rows()*(midPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(midPos[0]+rectWidth/2),
                            input.rows()*(midPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(//5-7
                    all,
                    new Point(
                            input.cols()*(rightPos[0]-rectWidth/2),
                            input.rows()*(rightPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(rightPos[0]+rectWidth/2),
                            input.rows()*(rightPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);

            switch (stageToRenderToViewport)
            {
                case THRESHOLD:
                {
                    return thresholdMat;
                }

                case detection:
                {
                    return all;
                }

                case RAW_IMAGE:
                {
                    return input;
                }

                default:
                {
                    return input;
                }
            }
        }

    }
}
