package org.firstinspires.ftc.teamcode.exp;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.apache.commons.math3.analysis.function.Power;
import org.firstinspires.ftc.teamcode.storage.PowerStorage;

import java.util.Timer;

@Autonomous(name = "PowerFollower", group = "Concept")
public class PowerFollower extends LinearOpMode {

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

        ElapsedTime timerA = new ElapsedTime();

        waitForStart();

        if (isStopRequested()) {
            return;
        };

        for(int i =0; i< PowerStorage.rightFrontPowers.size(); i++) {
            timerA.reset();
            leftFront.setPower(PowerStorage.leftFrontPowers.get(i));
            leftRear.setPower(PowerStorage.leftRearPowers.get(i));
            rightRear.setPower(PowerStorage.rightRearPowers.get(i));
            rightFront.setPower(PowerStorage.rightFrontPowers.get(i));

            sleep((long)0.05);
        }

        double clockSpeed = timerA.time();

        while(opModeIsActive()) {
            telemetry.addData("ClockCycle", clockSpeed);
            telemetry.update();
        }
    }
}
