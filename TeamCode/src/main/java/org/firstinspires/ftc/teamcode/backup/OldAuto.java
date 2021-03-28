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

package org.firstinspires.ftc.teamcode.backup;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Disabled
@Autonomous(name = "OldAuto", group = "Concept")

public class OldAuto extends LinearOpMode {

    BNO055IMU gyro;
    Orientation angles = new Orientation();

    public DcMotor front_left  = null;
    public DcMotor front_right = null;
    public DcMotor back_left   = null;
    public DcMotor back_right  = null;
    public DcMotorEx right_encoder  = null;

    private double lastAngle = 0;

    ElapsedTime tankTime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 383.6;
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;
    static final double     WHEEL_DIAMETER_INCHES   = 2.953;
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.14592);


    static final double     DRIVE_SPEED             = 0.2;
    static final double     TURN_SPEED              = 0.3;

    static final double     HEADING_THRESHOLD       = 1 ;
    static final double     P_TURN_COEFF            = 0.1;
    static final double     P_DRIVE_COEFF           = 0.05;

    @Override
    public void runOpMode() {

        //Gyro Init
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode

        telemetry.addData("Last Angle:", lastAngle);
        gyro = hardwareMap.get(BNO055IMU.class, "imu");

        //init
        front_left      = hardwareMap.get(DcMotor.class, "front_left");
        front_right     = hardwareMap.get(DcMotor.class, "front_right");
        back_left       = hardwareMap.get(DcMotor.class, "back_left");
        back_right      = hardwareMap.get(DcMotor.class, "back_right");
        right_encoder      = hardwareMap.get(DcMotorEx.class, "right_encoder");


        /*front_left.setMotorEnable();
        front_right.setMotorEnable();
        back_left.setMotorEnable();
        back_right.setMotorEnable();*/


        // Ensure the robot it stationary, then reset the encoders and calibrate the gyro.
        front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


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

        front_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        front_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Wait for the game to start (Display Gyro value), and reset gyro before we move..
        while (!isStarted()) {
            angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("Robot Heading = ", angles.firstAngle);
            telemetry.update();
        }

        /*Drive Code Goes Here*/
        driveInch(-10,0.3);
        turnDeg(90, 0.5);

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

        if (opModeIsActive()) {

            int worldXPosition = back_left.getCurrentPosition();
            int worldYPositionLeft = front_left.getCurrentPosition();
            int worldYPositionRight = right_encoder.getCurrentPosition();;


            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * COUNTS_PER_INCH);
            if (distance < 0 ) {
                customSpeed = -customSpeed;
            }
            newFrontLeftTarget = worldYPositionLeft - moveCounts;
            newFrontRightTarget = worldYPositionRight + moveCounts;
            newBackLeftTarget = worldYPositionLeft - moveCounts;
            newBackRightTarget = worldYPositionRight + moveCounts;

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
                    double adjustspeed = newspeed*((newFrontLeftTarget-back_left.getCurrentPosition())/1000);
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

            int worldXPosition = back_left.getCurrentPosition();
            int worldYPositionLeft = front_left.getCurrentPosition();
            int worldYPositionRight = right_encoder.getCurrentPosition();;

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * COUNTS_PER_INCH);
            if (distance < 0 ) {
                customSpeed = -customSpeed;
            }
            newFrontLeftTarget = worldYPositionLeft - moveCounts;
            newFrontRightTarget = worldYPositionRight + moveCounts;
            newBackLeftTarget = worldYPositionLeft - moveCounts;
            newBackRightTarget = worldYPositionRight + moveCounts;

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
                    double adjustspeed = newspeed*((newFrontLeftTarget-back_left.getCurrentPosition())/1000);
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
            while (opModeIsActive()) {
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

    public void turnDeg (  double angle, double speed) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
        lastAngle = getTrueAngle(angle);
        telemetry.addData("Last Angle:", lastAngle);
    }

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

    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }
}
