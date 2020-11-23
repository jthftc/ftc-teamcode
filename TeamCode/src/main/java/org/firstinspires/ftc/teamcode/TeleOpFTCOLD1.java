import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorController;
import java.lang.reflect.Field;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.hardware.CRServo;

/**
 * This is an example minimal implementation of the mecanum drivetrain
 * for demonstration purposes.  Not tested and not guaranteed to be bug free.
 *
 * @edited by Um Drug & Lick Sharty 
 * A little bit by Keenie Weenie Lemon Squeezie
 *
 */
 
@TeleOp(name="FTC2018-19", group="Iterative Opmode")
public class TeleOpFTCOLD1 extends OpMode {
    /*
     * The mecanum drivetrain involves four separate motors that spin in
     * different directions and different speeds to produce the desired
     * movement at the desired speed.
     */

    // declare and initialize four DcMotors.
    private DcMotor front_left  = null;
    private DcMotor front_right = null;
    private DcMotor back_left   = null;
    private DcMotor back_right  = null;
    private DcMotor left_arm = null;
    private DcMotor extender = null;
    private DcMotor flipper = null;
    private static int leftarmlimit = -1;
    private static int extenderlimit = -1;
    private static double multiplier = 1;
    private static double multiplierr = -1;
    private Servo santa = null;
    //private static boolean flipperbool = true;
    private DcMotor lift = null;
    private Servo trapdoor = null;
        @Override
    public void init() {
        // Name strings must match up with the config on the Robot Controller
        // app.
        front_left      = hardwareMap.get(DcMotor.class, "front_left");
        front_right     = hardwareMap.get(DcMotor.class, "front_right");
        back_left       = hardwareMap.get(DcMotor.class, "back_left");
        back_right      = hardwareMap.get(DcMotor.class, "back_right");
        left_arm        = hardwareMap.get(DcMotor.class, "left_arm");
        extender        = hardwareMap.get(DcMotor.class, "extender");
        lift            = hardwareMap.get(DcMotor.class, "lift");
        extenderlimit   = extender.getCurrentPosition();
        flipper         = hardwareMap.get(DcMotor.class, "flipper");
        trapdoor     = hardwareMap.servo.get("trapdoor");
        santa     = hardwareMap.servo.get("santa");
        flipper.setPower(0);
        trapdoor.setPosition(0.3);
        santa.setPosition(0.6);
         front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        front_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {
        // Mecanum drive is controlled with three axes: drive (front-and-back),
        // strafe (left-and-right), and twist (rotating the whole chas sis).
        telemetry.addData("front_left",front_left.getCurrentPosition());
        telemetry.addData("front_right",front_right.getCurrentPosition());
        telemetry.addData("back_left",back_left.getCurrentPosition());
        telemetry.addData("back_right",back_right.getCurrentPosition());
        double drive  = gamepad1.right_stick_y;
        double strafe = gamepad1.right_stick_x;
        double twist  = gamepad1.left_stick_x;
        float powerr  = gamepad2.left_stick_y;
        float powerrr = gamepad2.right_stick_y;
        boolean powerrrr = gamepad1.dpad_down;
        boolean powerrrrr = gamepad1.dpad_up;
           
            //if (flipperbool == true) {
              //  flipperpower = 0;
            //}
            //else {
              //  flipperpower = 1;
              
            if (gamepad1.left_trigger > 0) {
                santa.setPosition(-0.6);
            }
            else if (gamepad1.right_trigger > 0) {
                santa.setPosition(0.6);
            }
            
            if (gamepad2.left_trigger > 0) {
                trapdoor.setPosition(0.4);
            }
            else if (gamepad2.right_trigger > 0) {
                trapdoor.setPosition(-0.4);
            }
            if (gamepad2.right_bumper) {
                flipper.setPower(1);
            }
            else if (gamepad2.left_bumper) {
                flipper.setPower(0);
            }
            
            if (gamepad2.a) {
                multiplierr = -1;
            }
            if (gamepad2.b) {
                multiplierr = -0.8;
            }
                
            if (gamepad1.a) {
                multiplier = 1;
            }
            if (gamepad1.b) {
                multiplier = 0.85;
            }
            if (gamepad1.y) {
                multiplier = 0.6;
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
        left_arm.setPower(powerr*multiplierr);
        extender.setPower(0.8*powerrr);
       
      if (powerrrr == true) {
        lift.setPower(1);
      }
      else { if (powerrrrr == true) {
        lift.setPower(-1);
      }
      else {
        lift.setPower(0);
      }}
        int positionarm = left_arm.getCurrentPosition();
        int liftlimit = lift.getCurrentPosition();
        int positionextender = extender.getCurrentPosition();
        telemetry.addData("///////////FTC 2018-2019////////////","");
        telemetry.addData("Front Left Wheel Port:",front_left.getPortNumber());
        telemetry.addData("Front Right Wheel Port:",front_right.getPortNumber());
        telemetry.addData("Back Left Wheel Port:",back_left.getPortNumber());
        telemetry.addData("Back Right Wheel Port:",back_right.getPortNumber());
        telemetry.addData("Lift Port:",lift.getPortNumber());
        telemetry.addData("Extender Port:",extender.getPortNumber());
        telemetry.addData("Left Arm Port:",left_arm.getPortNumber());
        telemetry.addData("///////////JAVA THE HUTTS///////////","");
            }
        }
