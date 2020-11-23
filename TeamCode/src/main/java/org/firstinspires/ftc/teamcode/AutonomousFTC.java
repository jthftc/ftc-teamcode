/* Copyright (c) 2018 FIRST. All rights reserved.
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

package org.firstinspires.ftc.SEASON1;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

/**
 * This 2018-2019 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the gold and silver minerals.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@Autonomous(name = "BoxAutoFTC", group = "Concept")

public class AutonomousFTC extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY = "AepAbMv/////AAABmRGVNg4fdUxina2B1Jz3k+hUk491O0eNAtpdcDb3Bseqn3GGfDsSpQacuSuLyG4ROahR/y4G8EgGfZBKz55UJ6K941STVdEFyHE1Qv90G5PxTiv+KRsVVnrArOxZOOc9feBI9H/HUr/Y7/7vu50BCinB/9NIjGecgT2gum/FZfbSvbH6dDnGproIVOGslCXzI/FDrGD2YswyTg0x0Y9JhZBpxTk+Vc9hIrP5b2lUTf0a7QypUhKoXFFJm+LjltH5eTU62bTUNqgcjrTpnq7x0nFmAkVWjfcCj/96RVUROjgBgo7o3qyNKVX/87XXlXeTMqzjWgi4M4ZKkxrUKoZvx8riR8/XOuuCLgexTm5NrTLr";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
        private TFObjectDetector tfod;
        private DcMotor front_left  = null;
        private DcMotor front_right = null;
        private DcMotor back_left   = null;
        private DcMotor back_right  = null;
        private DcMotor lift        = null;
        private DcMotor left_arm    = null;
        private Servo   santa       = null;
        private DcMotor extender = null;
        private DcMotor flipper = null;
        private int sensorvalue = 0;
        private double frw = 1;
        private double back = -1;
        private double left = 1;
        private double right = -1;
        private int countertag = 0;
        
    public void drive(double powervar1,double powervar2,double powervar3,double powervar4,int sleeptime ) {
        front_left.setPower(powervar1);
        front_right.setPower(powervar2);
        back_left.setPower(powervar3);
        back_right.setPower(powervar4);
        sleep(sleeptime);
        front_left.setPower(0);
        front_right.setPower(0);
        back_left.setPower(0);
        back_right.setPower(0);
    }
    public void driveinch(int inch) {
        int mult = -1;
        front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        front_left.setTargetPosition((int)Math.round(-44.6186*inch));
        front_right.setTargetPosition((int)Math.round(44.6186*inch));
        back_left.setTargetPosition((int)Math.round(-44.6186*inch));
        back_right.setTargetPosition((int)Math.round(44.6186*inch));
        while(front_left.isBusy() && front_right.isBusy() && back_left.isBusy() && back_right.isBusy() && opModeIsActive()) {
        front_left.setPower(0.2);
        front_right.setPower(0.2);
        back_left.setPower(0.2);
        back_right.setPower(0.2);
        }
        front_left.setPower(0);
        front_right.setPower(0);
        back_left.setPower(0);
        back_right.setPower(0);
    }
    public void turndegree(int degree) { //default left
        int mult = -1;
        front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        front_left.setTargetPosition((int)Math.round(9.866*degree));
        front_right.setTargetPosition((int)Math.round(9.866*degree));
        back_left.setTargetPosition((int)Math.round(9.866*degree));
        back_right.setTargetPosition((int)Math.round(9.866*degree));
        while(front_left.isBusy() && front_right.isBusy() && back_left.isBusy() && back_right.isBusy() && opModeIsActive()) {
        front_left.setPower(0.2);
        front_right.setPower(0.2);
        back_left.setPower(0.2);
        back_right.setPower(0.2);
        }
        front_left.setPower(0);
        front_right.setPower(0);
        back_left.setPower(0);
        back_right.setPower(0);
    }
    public void tank(double powercus,int sleeptime ) {
        front_left.setPower(powercus*-1);
        front_right.setPower(powercus);
        back_left.setPower(powercus*-1);
        back_right.setPower(powercus);
        sleep(sleeptime);
        front_left.setPower(0);
        front_right.setPower(0);
        back_left.setPower(0);
        back_right.setPower(0);
    }
    public void strafe(double powercus,int sleeptime ) {
        front_left.setPower(powercus);
        front_right.setPower(powercus);
        back_left.setPower(powercus*-1);
        back_right.setPower(powercus*-1);
        sleep(sleeptime);
        front_left.setPower(0);
        front_right.setPower(0);
        back_left.setPower(0);
        back_right.setPower(0);
    }
    public void turn(double powercus,int sleeptime ) {
        front_left.setPower(powercus);
        front_right.setPower(powercus);
        back_left.setPower(powercus);
        back_right.setPower(powercus);
        sleep(sleeptime);
        front_left.setPower(0);
        front_right.setPower(0);
        back_left.setPower(0);
        back_right.setPower(0);
    }
    @Override
    public void runOpMode() {
    
//MOVEMENT AUTONOMOUS
     
        front_left   = hardwareMap.get(DcMotor.class, "front_left");
        front_right  = hardwareMap.get(DcMotor.class, "front_right");
        back_left    = hardwareMap.get(DcMotor.class, "back_left");
        back_right   = hardwareMap.get(DcMotor.class, "back_right");
        lift         = hardwareMap.get(DcMotor.class, "lift");
        left_arm     = hardwareMap.get(DcMotor.class, "left_arm");
        extender     = hardwareMap.get(DcMotor.class, "extender");
        flipper      = hardwareMap.get(DcMotor.class, "flipper");
        santa     = hardwareMap.get(Servo.class, "santa");
        santa.setPosition(0.6);
        
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            /** Activate Tensor Flow Object Detection. */
            if (tfod != null) {
                tfod.activate();
            }
        
            while (sensorvalue == 0 && countertag < 1000) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                      telemetry.addData("# Object Detected", updatedRecognitions.size());
                      if (updatedRecognitions.size() == 3) {
                        int goldMineralX = -1;
                        int silverMineral1X = -1;
                        int silverMineral2X = -1;
                        for (Recognition recognition : updatedRecognitions) {
                          if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                            goldMineralX = (int) recognition.getLeft();
                          } else if (silverMineral1X == -1) {
                            silverMineral1X = (int) recognition.getLeft();
                          } else {
                            silverMineral2X = (int) recognition.getLeft();
                          }
                        }
                        if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                          if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                            telemetry.addData("Gold Mineral Position", "Left");
                            sensorvalue = 1;
                          } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                            telemetry.addData("Gold Mineral Position", "Right");
                            sensorvalue = 3;
                          } else {
                            telemetry.addData("Gold Mineral Position", "Center");
                            sensorvalue = 2;
                          }
                        }
                      }
                      telemetry.update();
                      sleep(10);
                      countertag = ++countertag;
                    }
                }
            }
            
            if (countertag >= 1000) {
                sensorvalue = 4;
            }
            ////////////////////////////////////////////////////////////////////
            //Get Left Arm Out Of The Way
            left_arm.setPower(-1); 
            sleep(50);
            left_arm.setPower(0);
            sleep(100);
            //Strafe Right For 0.2 Seconds
            
            //Deattach From Lander
            lift.setPower(-1);
            sleep(2400);
            lift.setPower(0);
            sleep(600);
        
            //Straffing For 0.2 Seconds
            strafe(left,225);
            sleep(400);
            
            ////////////////////////////////////////////////////////////////////
            if (sensorvalue == 1){
            //Driving Forward
            driveinch(11);
           
            sleep(100); 
            
            //45 Degree Left Turn
            turndegree(52);
            
            sleep(100);
            
            //Driving For 0.15 Seconds
            driveinch(12);
            
            sleep(100);
            
            left_arm.setPower(-1); 
            sleep(700);
            left_arm.setPower(0);
            sleep(100);
            
            driveinch(-12);
            
            sleep(100);
            
             //Lift arm
            // Drive Forward
            driveinch(27);
            
            turndegree(-105);
            
            sleep(100);
            //Driving Back For 0.15 Seconds
            
            driveinch(23);
            
            sleep(300);
            //Turn 80 Degrees Right
            turndegree(-72);
            
            sleep(300);
            
            santa.setPosition(-0.6);
            
            sleep(500);
            
            //Close Santa 
            santa.setPosition(0.6);
            sleep(100);
            
            turndegree(-114);
            
            sleep(200);
            
            driveinch(58);
            
            extender.setPower(-1);
            sleep(1500);
            extender.setPower(0);
            
            lift.setPower(1);
            sleep(2400);
            lift.setPower(0);
            }
            ////////////////////////////////////////////////////////////////////
            if (sensorvalue == 2){
            
            driveinch(20);
            
            sleep(100);
            
            driveinch(-9);
            
            sleep(100);
            
            strafe(right,225);
            sleep(100);
            
             //90 Degree Left Turn
            left_arm.setPower(-1); 
            sleep(700);
            left_arm.setPower(0);
            sleep(100);
            
            //Driving For 0.2 Seconds
            driveinch(32);
           
            sleep(100); 
            
            //strafe(left, 200);
            turndegree(-90);
            
            sleep(200);
            
            //Drop Santa
            santa.setPosition(-0.6);
            
            sleep(500);
            
            //Close Santa 
            santa.setPosition(0.6);
            sleep(100);
            
            turndegree(180);
            
            //Strafe Right For 0.2 Seconds
            sleep(100);
            
            driveinch(17);
            
            sleep(100);
            
            turndegree(45);
            
            sleep(100);
            
            driveinch(60);
            
            extender.setPower(-1);
            sleep(1500);
            extender.setPower(0);
            
            lift.setPower(1);
            sleep(2400);
            lift.setPower(0);
            }
            ////////////////////////////////////////////////////////////////////
            if (sensorvalue == 3){
            //Driving Forward
            driveinch(13);
           
            sleep(100); 
            
            //90 Degree Right Turn
            turndegree(-92);
            
            sleep(100);
            
            //Driving Forward
            driveinch(15);
            
            sleep(100);
            
            turndegree(90);
            
            sleep(100);
            
            driveinch(9);
            
            sleep(100);
            
            left_arm.setPower(-1); 
            sleep(700);
            left_arm.setPower(0);
            sleep(100);
            
            driveinch(-9);
            
            sleep(100);
            
            driveinch(31);
            
            sleep(100);
            
            turndegree(-75);
            
            sleep(100);
            
            santa.setPosition(-0.6);
            
            sleep(500);
            
            //Close Santa 
            santa.setPosition(0.6);
            
            sleep(100);
            
            turndegree(165);
            
            sleep(100);
            
            driveinch(34);
            
            sleep(300);
            
            turndegree(43);
            
            sleep(100);
            
            driveinch(51);
            
            extender.setPower(-1);
            sleep(1500);
            extender.setPower(0);
            
            lift.setPower(1);
            sleep(2400);
            lift.setPower(0);
            
        }
        if (sensorvalue == 4) {
            driveinch(20);
            
            sleep(100);
            
            driveinch(-9);
            
            sleep(100);
            
            strafe(right,225);
            sleep(100);
            
             //90 Degree Left Turn
            left_arm.setPower(-1); 
            sleep(700);
            left_arm.setPower(0);
            sleep(100);
            
            //Driving For 0.2 Seconds
            driveinch(32);
           
            sleep(100); 
            
            //strafe(left, 200);
            turndegree(-90);
            
            sleep(200);
            
            //Drop Santa
            santa.setPosition(-0.6);
            
            sleep(500);
            
            //Close Santa 
            santa.setPosition(0.6);
            sleep(100);
            
            turndegree(180);
            
            //Strafe Right For 0.2 Seconds
            sleep(100);
            
            driveinch(17);
            
            sleep(100);
            
            turndegree(45);
            
            sleep(100);
            
            driveinch(60);
            
            extender.setPower(-1);
            sleep(1500);
            extender.setPower(0);
            
            lift.setPower(1);
            sleep(2400);
            lift.setPower(0);
        }

        if (tfod != null) {
            tfod.shutdown();
        }
      stop();  
    }
}
    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Cam");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }
}
