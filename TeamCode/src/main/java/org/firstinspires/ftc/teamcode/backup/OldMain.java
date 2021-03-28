package org.firstinspires.ftc.teamcode.backup;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


/**
 *
 * Implemented by Hutt Technologies Group.
 *
 */
@Disabled
@TeleOp(name="OldMain", group="Iterative Opmode")
public class OldMain extends OpMode {

    //Declare Gyro
    BNO055IMU gyro;
    Orientation angles = new Orientation();

    // Declare the DC Motors
    private DcMotorEx front_left  = null;
    private DcMotorEx front_right = null;
    private DcMotorEx back_left   = null;
    private DcMotorEx back_right  = null;
    private DcMotor gripper = null;
    private DcMotor shooter1 = null;
    private DcMotor shooter2 = null;
    private DcMotorEx right_encoder  = null;

    private Servo cam_servo = null;


    private double multiplier = 1;

    private double leftFrontStop = 0;
    private double leftBackStop = 0;
    private double rightFrontStop = 0;
    private double rightBackStop = 0;

    private boolean relative = true;
    private boolean toggleShooter = false;

    //Declare Timers
    ElapsedTime holdTimerA = new ElapsedTime();
    ElapsedTime holdTimerB = new ElapsedTime();
    ElapsedTime holdTimerC = new ElapsedTime();
    ElapsedTime holdTimerD = new ElapsedTime();
    ElapsedTime holdTimerE = new ElapsedTime();
    ElapsedTime holdTimerF = new ElapsedTime();
    ElapsedTime stopTimer = new ElapsedTime();


    @Override
    public void init() {

        //Init Gyro
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        gyro = hardwareMap.get(BNO055IMU.class, "imu");
        gyro.initialize(parameters);

        //Init Motors Within the Hardware
        front_left      = hardwareMap.get(DcMotorEx.class, "front_left");
        front_right     = hardwareMap.get(DcMotorEx.class, "front_right");
        back_left       = hardwareMap.get(DcMotorEx.class, "back_left");
        back_right      = hardwareMap.get(DcMotorEx.class, "back_right");
        gripper         = hardwareMap.get(DcMotor.class, "gripper");
        shooter1         = hardwareMap.get(DcMotor.class, "shooter1");
        shooter2         = hardwareMap.get(DcMotor.class, "shooter2");
        right_encoder      = hardwareMap.get(DcMotorEx.class, "right_encoder");

        cam_servo       = hardwareMap.get(Servo.class, "cam_servo");

        //Set Encoder Methods
        front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        gripper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        front_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        front_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        gripper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        front_left.setMotorEnable();
        front_right.setMotorEnable();
        back_left.setMotorEnable();
        back_right.setMotorEnable();

        cam_servo.setPosition(0);

        shooter1.setPower(0);
        shooter2.setPower(0);


    }
    
    
    @Override
    public void loop() {

        telemetry.addData("///////////FTC 2020-2021////////////","");

        telemetry.addData("Front Left Wheel Port:",front_left.getPortNumber());
        telemetry.addData("Front Right Wheel Port:",front_right.getPortNumber());
        telemetry.addData("Back Left Wheel Port:",back_left.getPortNumber());
        telemetry.addData("Back Right Wheel Port:",back_right.getPortNumber());

        telemetry.addData("Front Left Power:",front_left.getPower());
        telemetry.addData("Front Right Power:",front_right.getPower());
        telemetry.addData("Back Left Power:",back_left.getPower());
        telemetry.addData("Back Right Power:",back_right.getPower());

        telemetry.addData("World X Position",back_left.getCurrentPosition());
        telemetry.addData("World Y Position Left",front_left.getCurrentPosition());
        telemetry.addData("World Y Position Right",right_encoder.getCurrentPosition());

        telemetry.addData("Gripper Position",gripper.getCurrentPosition());

        telemetry.addData("Gamepad Right Y Position",gamepad1.right_stick_y);
        telemetry.addData("Gamepad Right X Position",gamepad1.right_stick_x);
        telemetry.addData("Gamepad Left X Position",gamepad1.left_stick_x);

        telemetry.addData("timer",holdTimerA.time());

        telemetry.addData("///////////JAVA THE HUTTS///////////","");

        angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double drive = gamepad1.right_stick_y;
        double strafe = gamepad1.right_stick_x;
        double twist  = gamepad1.left_stick_x;

        if (relative == true) {
            telemetry.addData("Orginal Drive", drive);
            telemetry.addData("Orginal Strafe", strafe);

            double gyro_degrees = angles.firstAngle;
            double gyro_radians = gyro_degrees * Math.PI / 180;

            double temp = drive * Math.cos(gyro_radians) + strafe * Math.sin(gyro_radians);
            strafe = -drive * Math.sin(gyro_radians) + strafe * Math.cos(gyro_radians);
            drive = temp;

            telemetry.addData("New Drive", drive);
            telemetry.addData("New Strafe", strafe);
        }


        double gripperPower = 0;
        //if (gripper.getCurrentPosition() <= 300) {
            gripperPower += gamepad1.right_trigger;
        //}
        //if (gripper.getCurrentPosition() >= 0) {
            gripperPower -= gamepad1.left_trigger;
        //}

        //Shooter Toggle
        if (gamepad1.a == true) {
            if ((toggleShooter == true) && (holdTimerB.time() > 0.5)) {
                shooter1.setPower(1);
                shooter2.setPower(1);
                toggleShooter = false;
                holdTimerB.reset();
            } else if ((toggleShooter == false) && (holdTimerB.time() > 0.5)) {
                shooter1.setPower(0);
                shooter2.setPower(0);
                toggleShooter = true;
                holdTimerB.reset();
            }
        }

        int worldXPosition = back_left.getCurrentPosition();
        int worldYPositionLeft = front_left.getCurrentPosition();
        int worldYPositionRight = right_encoder.getCurrentPosition();

        if (gamepad1.b) {
            multiplier = 1;
        }
        if (gamepad1.y) {
            multiplier = 0.4;
        }

        if (gamepad1.x) {
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
            gyro = hardwareMap.get(BNO055IMU.class, "imu");
            gyro.initialize(parameters);

        }

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

        front_left.setPower(speeds[0]*multiplier );
        front_right.setPower(speeds[1]*multiplier );
        back_left.setPower(speeds[2]*multiplier );
        back_right.setPower(speeds[3]*multiplier );

        gripper.setPower(gripperPower*0.6);
        
    }
}
