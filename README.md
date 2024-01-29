
# Greetings! We are Team Vietnam

## What is this?
The GitHub repository contains the source code that was used in the FIRST Global Challenge 2022. It was written based on the [public FTC SDK](https://github.com/FIRST-Tech-Challenge/FtcRobotController). To use this program, please download/clone the entire project to your local computer.

## What it does?
This source code is used to build custom op modes and incorporate them into the FTC RobotController app to control Team Vietnam's FGC 2022 robot. 

## Requirements
### Software requirements
To use this project, please first download [Android Studio](https://developer.android.com/studio) to your local computer.

This program is written in the Java language and built by Gradle. 

* To install Java on your local computer, please download the compatible JDK version on Java's official website, or you can click the link below:

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[Java installation](https://www.oracle.com/java/technologies/downloads/)

Notes: it is recommended to download the latest version, which is 21.0.2.

* To install Gradle, please visit the link below:

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[Gradle package](https://gradle.org/releases/) 

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[Gradle installation guide](https://gradle.org/install/) 

For more information on Java and Gradle, please check out the Supported Documents sections.

### Hardware requirements
This program is built for [FTC Control System](https://ftc-docs.firstinspires.org/en/latest/programming_resources/shared/control_system_intro/The-FTC-Control-System.html).

### Language requirements
Because this program is written in Java, please prepare yourself with some basic understanding of Java's syntax to understand and use this program. In the supported documents section, there are links to online Java's tutorials if needed.

If you are new to robotics or new to the *FIRST* Tech Challenge, then you should consider reviewing the [FTC Blocks Tutorial](https://github.com/FIRST-Tech-Challenge/FtcRobotController/wiki/Blocks-Tutorial) to get familiar with how to use the control system:

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[FTC Blocks Online Tutorial](https://github.com/FIRST-Tech-Challenge/FtcRobotController/wiki/Blocks-Tutorial)

Even if you are an advanced Java programmer, it is helpful to start with the [FTC Blocks tutorial](https://github.com/FIRST-Tech-Challenge/FtcRobotController/wiki/Blocks-Tutorial), and then migrate to the [OnBot Java Tool](https://github.com/FIRST-Tech-Challenge/FtcRobotController/wiki/OnBot-Java-Tutorial) or to [Android Studio](https://github.com/FIRST-Tech-Challenge/FtcRobotController/wiki/Android-Studio-Tutorial) afterwards.

### For advanced developers
This program contains a developing version that implements localization to locate the position of the robot on the FGC 2022's game field. It makes use of an external library to calculate paths. To install this library, please visit [the FTC lib installation guide](https://docs.ftclib.org/ftclib/v/v2.0.0/installation).

### Notice!
* Remember to choose the correct version based on your platform (Mac, Windows).

## Downloading this project
There are many ways you can use to download this repo:

* If you are a git user, you can clone the most current version of the repository by pasting the below command to your terminal:
<p>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;git clone https://github.com/TeamVietnam/FGC-2022.git</p>

* If you prefer, you can use the "Download Zip" button available through the main repository page. Downloading the project as a .ZIP file will keep the size of the download manageable.

Once you have downloaded and uncompressed (if needed) your folder, you can use Android Studio to import the folder  ("Import project (Eclipse ADT, Gradle, etc.)").

## File structure

![Alt](/TeamCode/imglib/TeamCode.png)

In the [TeamCode module](/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/), our main program is written and divided into 5 main categories: 

![Alt](/TeamCode/imglib/File_structure.png)

* OpModes: this part contains op modes files (Main.java and Test.java), which contain a series of instructions for the robot's behaviors. While the Main.java file was mainly used to compete during the FIRST Global Challenge 2022, Test.java was used to test our localization program.
![Alt](/TeamCode/imglib/OpMode.png)

* Subsystems: a subsystem contains a collection of the robot's hardware that functions as a unit. For example, our drivebase (or chassis) is a combination of 4 active DC motors.
![Alt](/TeamCode/imglib/Subsystems.png)

* Robot: instead of writing directly to the op mode file, we decided to describe the behaviors of our robot or reactions to events on a separate file. Each file in the Robot module contains subsystems required for a functional robot (or to test certain features) for each op mode. Robot.java was used with Main op mode, while TestRobot.java was used with Test op mode.
![Alt](/TeamCode/imglib/Robot.png)

* utils: this part contains supported programs to calculate paths, ball projectiles, trajectories, etc.
![Alt](/TeamCode/imglib/utils.png)

* Constants: this file contains constants. For example, instead of rewriting the velocity for many mechanisms having the same speed, we could just keep the speed's value in this file (double speed = 0.5).
![Alt](/TeamCode/imglib/Constants.png)

In the [FtcRobotController module](/FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples), you can find several sample op modes, which can be pasted to your TeamCode module to use as-is. 

For more information, please visit the Supported documents section. 

## Getting started

### Hardware configuration 
Please follow [this guide](https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/index.html) for hardware and software configuration. 

After finishing the program, to build the robot code with Gradle (or you can interpret it as uploading the program to the FTC Robot Controller app), please follow these steps:

* Step 1: plug the USB-C cable from your computer to the Control Hub.

* Step 2: when the connection is established, on the top toolbar, the name of the connected device would prompt next to the green triangle button (run button). 

![Alt](/TeamCode/imglib/Device_name.png)

Press the green triangle button to build the Robot Controller app and to install it on your control device.

![Alt](/TeamCode/imglib/toolbar.png)

* Step 3: if this program is built successfully, there would be a green notification that the launch has been successful. 

For more information on how to launch the program, please visit Supported documents section. 

## Supported documents
### User Documentation and Tutorials
* If you don't have or has a poor **Java background**:

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[Java W3 School tutorials](https://www.w3schools.com/java/)

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[Java Geeks for Geeks tutorial](https://www.geeksforgeeks.org/java/)

* If you want to master **Gradle and its structure**: 

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[Gradle User Manual](https://docs.gradle.org/current/userguide/userguide.html)

* If you want to understand more about **Android Studio and its capabilities**:

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[Android Studio Basics](https://www.youtube.com/watch?v=lNKk-RSL7wg)

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[Android Developer Courses](https://developer.android.com/courses/android-basics-compose/unit-1)

* If you want to master **Git**: 

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[Github Basics](https://www.youtube.com/watch?v=1JuYQgpbrW0&t=895s)

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[Github Completed Usage](https://git-scm.com/book/en/v2) 

* If you want to understand more about **FTC programming**: 

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[FTC Programming](https://ftc-docs.firstinspires.org/en/latest/programming_resources/android_studio_java/opmode/opmode.html)

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[Launching the program with Android Studio](https://ftc-docs.firstinspires.org/en/latest/programming_resources/tutorial_specific/android_studio/creating_op_modes/Creating-and-Running-an-Op-Mode-%28Android-Studio%29.html)

### Javadoc Reference Information
The Javadoc reference documentation for the FTC SDK is now available online.

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[FTC Javadoc Documentation](https://javadoc.io/doc/org.firstinspires.ftc)

### Developers Forum
For technical questions, there are some forums you could join to ask questions:

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[FTC Technology Forum](https://ftcforum.firstinspires.org/forum/ftc-technology)

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[Stack Over Flow](https://stackoverflow.com/)

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[Chief Delphi](https://www.chiefdelphi.com/c/other/first-tech-challenge/60)

### Advanced materials

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[April Tag Programming](https://ftc-docs.firstinspires.org/en/latest/apriltag/vision_portal/apriltag_intro/apriltag-intro.html)

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[TensorFlow Programming](https://ftc-docs.firstinspires.org/en/latest/programming_resources/vision/tensorflow_cs_2023/tensorflow-cs-2023.html)

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[Vision Programming](https://ftc-docs.firstinspires.org/en/latest/programming_resources/vision/vision_overview/vision-overview.html)

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[PID Controller](https://docs.ftclib.org/ftclib/features/controllers)

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[Odometry Youtube series](https://www.youtube.com/watch?v=GEZBYHVHmFQ)

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[Kalman Filter](https://www.ctrlaltftc.com/advanced/the-kalman-filter)

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[FTC Lib](https://docs.ftclib.org/ftclib/v/v2.0.0/)

These two below links are for FIRST Robotics Competition, a different robotics competition, but they are still great examples to demonstrate each topic in details. 

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[PID Controller mechanism](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/introduction-to-pid.html)

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[Kalman Filters mechanism](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/state-space/state-space-observers.html)

