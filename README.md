# FTC TeamCode Repository 🤖

[![Website](https://img.shields.io/website?label=javathehutts.org&style=for-the-badge&url=http%3A%2F%2Fjavathehutts.org)](https://javathehutts.org)
[![Twitter Follow](https://img.shields.io/twitter/follow/javathehutts?color=1DA1F2&logo=twitter&style=for-the-badge)](https://twitter.com/intent/follow?original_referer=https%3A%2F%2Fgithub.com%2Fjavathehutts&screen_name=javathehutts)
[![Instagram Follow](https://img.shields.io/badge/instagram-%23E4405F.svg?&style=for-the-badge&logo=instagram&logoColor=white)](https://instagram.com/jthftc)

## Getting Started With Java

To start off with, you are going to need to download the current [FTC SDK](https://github.com/FIRST-Tech-Challenge/FtcRobotController) from the Official First Technology Challenge Github page. This will include a .zip file that includes all of the base structures that your team is going to need in order to develop a functional Java program.

Next, your going to need to either setup an [OnBotJava](https://github.com/FIRST-Tech-Challenge/FtcRobotController/wiki/OnBot-Java-Tutorial) or [Andriod Studio](https://github.com/FIRST-Tech-Challenge/FtcRobotController/wiki/Android-Studio-Tutorial) environment for editing and compiling your Java projects. The links in the previous sentence will bring you to the respective tutorials provided by First Robotics.  

## Developing an OpMode:

### Basic Template

To begin creating your first java program, open up your /teamcode folder within your [FTC SDK](https://github.com/FIRST-Tech-Challenge/FtcRobotController) and create a new file named ```main.java```. Next copy and paste the following template into this new java file. This template includes basic imports, class syntax, and the ```init()```/```loop()``` in which will help define our robot objectives during the TeleOp period.

```java
package org.firstinspires.ftc.teamcode; //This may change depending on where this file is stored

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp; 
import com.qualcomm.robotcore.hardware.DcMotor; //Needed for Motor Control
import com.qualcomm.robotcore.hardware.DcMotorEx; //Needed for Motor Control
import com.qualcomm.robotcore.hardware.Servo; //Needed for Servo Control
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="main", group="Opmode") //Name & group of OpMode within the Driver Station Phone
@Disabled //Remove when finished and ready for testing

public class main extends OpMode {

    @Override
    public void init() {
      //Initialization Code
    }
    
    @Override
    public void loop() {
      //Loop Code
    }

}
```
### Initializing Motors


### Controlling Motors


### Initializing Servos


### Controlling Servos


### Exporting OpMode


### Adjusting OpMode Based Upon Drivetrain



--- 

### Connect with us:

[<img align="left" alt="javathehutts.org" width="22px" src="https://raw.githubusercontent.com/iconic/open-iconic/master/svg/globe.svg" />][website]
[<img align="left" alt="javathehutts | Facebook" width="22px" src="https://cdn.jsdelivr.net/npm/simple-icons@v3/icons/facebook.svg" />][facebook]
[<img align="left" alt="javathehutts | Twitter" width="22px" src="https://cdn.jsdelivr.net/npm/simple-icons@v3/icons/twitter.svg" />][twitter]
[<img align="left" alt="javathehutts | Instagram" width="22px" src="https://cdn.jsdelivr.net/npm/simple-icons@v3/icons/instagram.svg" />][instagram]
[<img align="left" alt="javathehutts | YouTube" width="22px" src="https://cdn.jsdelivr.net/npm/simple-icons@v3/icons/youtube.svg" />][youtube]
[<img align="left" alt="javathehutts | Reddit" width="22px" src="https://cdn.jsdelivr.net/npm/simple-icons@v3/icons/reddit.svg" />][reddit]

<br />



---




[website]: http://javathehutts.org
[twitter]: https://twitter.com/javathehutts
[youtube]: https://youtube.com/channel/UC7lOdu9FJqzLBgwIap4CDhw
[instagram]: https://instagram.com/jthftc
[facebook]: https://www.facebook.com/Javathehutts/
[reddit]: https://www.reddit.com/user/JavaTheHutts/
