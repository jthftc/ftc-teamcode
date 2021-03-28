package org.firstinspires.ftc.teamcode.exp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.storage.PowerStorage;

import java.util.ArrayList;

@TeleOp(name="PowerDectector", group = "Iterative Opmode")

public class PowerDectector extends LinearOpMode {
    public DcMotorEx leftFront;
    public DcMotorEx leftRear;
    public DcMotorEx rightRear;
    public DcMotorEx rightFront;
    @Override
    public void runOpMode() throws InterruptedException {
        leftFront = hardwareMap.get(DcMotorEx.class, "front_left");
        leftRear = hardwareMap.get(DcMotorEx.class, "back_left");
        rightRear = hardwareMap.get(DcMotorEx.class, "back_right");
        rightFront = hardwareMap.get(DcMotorEx.class, "front_right");

        ArrayList<Double> leftFrontPowers = new ArrayList<Double>();
        ArrayList<Double> leftRearPowers = new ArrayList<Double>();
        ArrayList<Double> rightRearPowers = new ArrayList<Double>();
        ArrayList<Double> rightFrontPowers = new ArrayList<Double>();

        ArrayList<Double> time = new ArrayList<Double>();

        ElapsedTime timerA = new ElapsedTime();

        waitForStart();

        if (isStopRequested()) {
            return;
        };

        while (opModeIsActive() && !isStopRequested()) {
            telemetry.addData("ClockCycle", timerA.time());
            telemetry.update();
            timerA.reset();



            double drive = gamepad1.right_stick_y;
            double strafe = gamepad1.right_stick_x;
            double twist  = gamepad1.left_stick_x;

            double[] speeds = {
                    (drive - strafe - twist),
                    (-drive - strafe - twist),
                    (drive + strafe - twist),
                    (-drive + strafe - twist)
            };

            double max = Math.abs(speeds[0]);
            for(int i = 0; i < speeds.length; i++) {
                if ( max < Math.abs(speeds[i]) ) max = Math.abs(speeds[i]);
            }


            if (max > 1) {
                for (int i = 0; i < speeds.length; i++) speeds[i] /= max;
            }

            leftFront.setPower(speeds[0] );
            rightFront.setPower(speeds[1] );
            leftRear.setPower(speeds[2] );
            rightRear.setPower(speeds[3] );

            leftFrontPowers.add(speeds[0]);
            leftRearPowers.add(speeds[2]);
            rightRearPowers.add(speeds[3]);
            rightFrontPowers.add(speeds[1]);

            time.add(timerA.milliseconds());

        }

        PowerStorage.leftFrontPowers = leftFrontPowers;
        PowerStorage.leftRearPowers = leftRearPowers;
        PowerStorage.rightRearPowers = rightRearPowers;
        PowerStorage.rightFrontPowers = rightFrontPowers;

        PowerStorage.time = time;
    }
}
