package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.vision.UGRectRingPipeline;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@TeleOp
public class FTCLibVisionTest2 extends LinearOpMode {
    UGRectDetector UGRectDetector;

    @Override
    public void runOpMode() throws InterruptedException {

        UGRectDetector = new UGRectDetector(hardwareMap, "Cam");
        UGRectDetector.init();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setCameraPosition(0.5);

        waitForStart();

        while (opModeIsActive()) {
            UGRectDetector.Stack stack = UGRectDetector.getStack();
            switch (stack) {
                case ZERO:
                    telemetry.addData("[Ring Stack] >>", "ZERO");
                    telemetry.update();
                    break;
                case ONE:
                    telemetry.addData("[Ring Stack] >>", "ONE");
                    telemetry.update();
                    break;
                case FOUR:
                    telemetry.addData("[Ring Stack] >>", "FOUR");
                    telemetry.update();
                    break;
                default:
                    telemetry.addData("[Ring Stack] >>", "NONE");
                    telemetry.update();
                    break;
            }
        }


    }


}