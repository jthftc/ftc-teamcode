package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is an example minimal implementation of the mecanum drivetrain
 * for demonstration purposes.  Not tested and not guaranteed to be bug free.
 *
 * @edited by Aum Dhruv & Nick Harty
 * A little bit by Kyle Keene
 *
 */

@TeleOp(name="FTC2019-2020", group="Iterative Opmode")
public class Main extends OpMode {

    // Declare the DC Motors
    private DcMotor front_left  = null;
    private DcMotor front_right = null;
    private DcMotor back_left   = null;
    private DcMotor back_right  = null;
    private DcMotor left_lift = null;
    private DcMotor right_lift = null;
    private DcMotor left_spin = null;
    private DcMotor right_spin = null;
    private Servo gripper1 = null;
    private Servo gripper2 = null;
    private Servo grabber = null;
    private Servo left_intake = null;
    private Servo right_intake = null;
    private Servo adjustArm = null;
    private Servo pullArm = null;
    private double multiplier = 1;
    private double multlift = 1;
    private boolean lastState = false;
    private boolean initGrab = true;
    private boolean swIntake = false;
    private boolean togglePull = false;
    ElapsedTime holdTimerA = new ElapsedTime();
    ElapsedTime holdTimerB = new ElapsedTime();
    ElapsedTime holdTimerC = new ElapsedTime();
    ElapsedTime holdTimerD = new ElapsedTime();
    ElapsedTime holdTimerE = new ElapsedTime();
    ElapsedTime holdTimerF = new ElapsedTime();

    //Init
    @Override
    public void init() {

        //Init Motors Within the Hardware
        front_left      = hardwareMap.get(DcMotor.class, "front_left");
        front_right     = hardwareMap.get(DcMotor.class, "front_right");
        back_left       = hardwareMap.get(DcMotor.class, "back_left");
        back_right      = hardwareMap.get(DcMotor.class, "back_right");
        left_lift       = hardwareMap.get(DcMotor.class, "left_lift");
        right_lift      = hardwareMap.get(DcMotor.class, "right_lift");
        left_spin       = hardwareMap.get(DcMotor.class, "left_spin");
        right_spin      = hardwareMap.get(DcMotor.class, "right_spin");
        gripper1        = hardwareMap.get(Servo.class, "gripper1");
        gripper2        = hardwareMap.get(Servo.class, "gripper2");
        left_intake        = hardwareMap.get(Servo.class, "left_intake");
        right_intake        = hardwareMap.get(Servo.class, "right_intake");
        grabber         = hardwareMap.get(Servo.class, "grabber");
        adjustArm       = hardwareMap.get(Servo.class, "adjustArm");
        pullArm         = hardwareMap.get(Servo.class,"pullArm");
        
        //Set Encoder Methods
        front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_spin.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_spin.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        front_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_spin.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_spin.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        gripper1.setPosition(0);
        gripper2.setPosition(1);
        left_intake.setPosition(0.4);//in
        right_intake.setPosition(0.4);
        pullArm.setPosition(0.84);
    }
    
    
    @Override
    public void loop() {
        telemetry.addData("///////////FTC 2019-2020////////////","");
        telemetry.addData("Front Left Wheel Port:",front_left.getPortNumber());
        telemetry.addData("Front Right Wheel Port:",front_right.getPortNumber());
        telemetry.addData("Back Left Wheel Port:",back_left.getPortNumber());
        telemetry.addData("Back Right Wheel Port:",back_right.getPortNumber());
        telemetry.addData("front_left",front_left.getCurrentPosition());
        telemetry.addData("front_right",front_right.getCurrentPosition());
        telemetry.addData("back_left",back_left.getCurrentPosition());
        telemetry.addData("back_right",back_right.getCurrentPosition());
        telemetry.addData("left_lift",left_lift.getCurrentPosition());
        telemetry.addData("right_lift",right_lift.getCurrentPosition());
        telemetry.addData("timer",holdTimerA.time());
        telemetry.addData("///////////JAVA THE HUTTS///////////","");
        if (initGrab == true) {
            grabber.setPosition(0.5);
            initGrab = false;
        }
        double drive  = gamepad1.right_stick_y;
        double strafe = gamepad1.right_stick_x;
        double twist  = gamepad1.left_stick_x*0.75;
        if (gamepad1.b) {
            multiplier = 1;
        }
        if (gamepad1.y) {
            multiplier = 0.4;
        }
        if (gamepad2.x) {
            multlift = 0.4;
        }
        if (gamepad2.a) {
            multlift = 1;
        }
        int currentLeftLift = left_lift.getCurrentPosition();
        int currentRightLift = right_lift.getCurrentPosition();
        if (gamepad2.dpad_up) {
            right_lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            left_lift.setPower(1*multlift);
            right_lift.setPower((1*multlift));
        }
        else if (gamepad2.dpad_down) {
            right_lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            left_lift.setPower(-1*multlift);
            right_lift.setPower((-1*multlift));
        }
        else {
            if (currentRightLift < currentLeftLift + 5 && currentRightLift > currentLeftLift-5) {
                right_lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                right_lift.setPower(0);
                left_lift.setPower(0);
            }
            else {
                right_lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                if (currentLeftLift-currentRightLift > 0) {
                    right_lift.setPower(0.5);
                }
                else {
                    right_lift.setPower(-0.5);
                }
                left_lift.setPower(0);
            }
        }
        if ((gamepad1.left_trigger > 0) && (holdTimerA.time() > 0.5)) {     
            holdTimerA.reset();
            
            if(left_spin.getPower() == -1 || Math.abs(left_spin.getPower()) == 0) {

                left_spin.setPower(0.5);
                right_spin.setPower(-0.5);
            }
            else {

                left_spin.setPower(0);
                right_spin.setPower(0);
            }
        }
        if ((gamepad1.right_trigger > 0) && (holdTimerB.time() > 0.5)) {
            holdTimerB.reset();
            
            if(left_spin.getPower() == 0.5 || Math.abs(left_spin.getPower()) == 0) {
                left_spin.setPower(-1);
               right_spin.setPower(1);
            }
            else {
                left_spin.setPower(0);
               right_spin.setPower(0);
            }
        }
        if ((gamepad1.x == true) && (holdTimerC.time() > 0.5)) {     
            holdTimerC.reset();

            if(swIntake) {
                left_intake.setPosition(0.5);//out
                right_intake.setPosition(0.35);
                swIntake = false;
            }
            else {
               left_intake.setPosition(0.4);//in
               right_intake.setPosition(0.4);
                swIntake = true;
            }
        }
        if ((gamepad1.a ==true ) && (holdTimerE.time() > 0.5)) {
            holdTimerE.reset();

            if(gripper1.getPosition() == 0) {

                gripper1.setPosition(1);
                gripper2.setPosition(0);
            }
            else {

                gripper1.setPosition(0);
                gripper2.setPosition(1);
            }
        }
        if ((gamepad2.right_bumper == true) && (holdTimerD.time() > 0.5)) {
            holdTimerD.reset();

            if (grabber.getPosition() == 0.5) {
                grabber.setPosition(1);

            } else {
                grabber.setPosition(0.5);
            }
        }
        if (gamepad2.right_trigger > 0) {
            adjustArm.setPosition(0.6);
        }
        else if (gamepad2.left_trigger > 0) {
            adjustArm.setPosition(0.1);
        }

        if ((gamepad2.y == true) && (holdTimerF.time() > 0.5)) {
            holdTimerF.reset();

            if (togglePull) {
                pullArm.setPosition(0);
                togglePull = false;
            } else {
                pullArm.setPosition(0.96);
                togglePull = true;
            }
        }
        /*
         * If we had a gyro and wanted to do field-oriented control, here
         * is where we would implement it.
         * The idea is fairly simple; we have a robot-oriented Cartesian (x,y)
         * coordinate (strafe, drive), and we just rotate it by the gyro
         * reading minus the offset that we read in the init() method.
         * Some rough pseudocode demonstrating:
         *         * if Field Oriented Control:
         *     get gyro heading
         *     subtract initial offset from heading
         *     convert heading to radians (if necessary)
         *     new strafe = strafe * cos(heading) - drive * sin(heading)
         *     new drive  = strafe * sin(heading) + drive * cos(heading)
         *
         * If you want more understanding on where these rotation formulas come
         * from, refer to
         * https://en.wikipedia.org/wiki/Rotation_(mathematics)#Two_dimensions
         */

        // You may need to multiply some of these by -1 to invert direction of
        // the motor.  This is not an issue with the calculations themselves.
        double[] speeds = {
                (drive - strafe - twist),
                (-drive - strafe - twist),
                (drive + strafe - twist),
                (-drive + strafe - twist)
        };

        // Because we are adding vectors and motors only take values between
        // [-1,1] we may need to normalize them.

        // Loop through all values in the speeds[] array and find the greatest
        // *magnitude*.  Not the greatest velocity.
        double max = Math.abs(speeds[0]);
        for(int i = 0; i < speeds.length; i++) {
            if ( max < Math.abs(speeds[i]) ) max = Math.abs(speeds[i]);
        }
        // If and only if the maximum is outside of the range we want it to be,
        // normalize all the other speeds based on the given speed value.
        if (max > 1) {
            for (int i = 0; i < speeds.length; i++) speeds[i] /= max;
        }

        // apply the calculated values to the motors.

        front_left.setPower(speeds[0]*multiplier);
        front_right.setPower(speeds[1]*multiplier);
        back_left.setPower(speeds[2]*multiplier);
        back_right.setPower(speeds[3]*multiplier);
        
    }
}
