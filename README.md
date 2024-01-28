
# Greetings! We are Team Vietnam

## Introduction
This GitHub repository contains the source code that was used in FIRST Global Challenge 2022. It was written based on the [public FTC SDK](https://github.com/FIRST-Tech-Challenge/FtcRobotController). To use this program, please download/clone the entire project to your local computer.

## Requirements
### Software requirements
To use this project, please first download the [Android Studio](https://developer.android.com/studio) to your local computer.

This program is written in Java language and built by Gradle. 

* To install Java on your local computer, please download the compatible JDK version on Java's official website or you can click the link below:

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[Java installation](https://www.oracle.com/java/technologies/downloads/)

Notes: it is recommended to download the latest version, which is 21.0.2.

* To install Gradle, please visit the link below:

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[Gradle package](https://gradle.org/releases/) (Downloading versions)

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[Gradle installation guide](https://gradle.org/install/) (Instructions on installation)

For more information on Java and Gradle, please visit the Supported Documents sections.

### Hardware requirements
This program is built for [FTC Control System](https://ftc-docs.firstinspires.org/en/latest/programming_resources/shared/control_system_intro/The-FTC-Control-System.html).

### Language requirements
Because this program is written in Java, please prepare yourself with some basic understandings of Java's syntax to understand and use this program. In the supported documents section, there are links to online Java's tutorials if needed.

If you are new to robotics or new to the *FIRST* Tech Challenge, then you should consider reviewing the [FTC Blocks Tutorial](https://github.com/FIRST-Tech-Challenge/FtcRobotController/wiki/Blocks-Tutorial) to get familiar with how to use the control system:

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[FTC Blocks Online Tutorial](https://github.com/FIRST-Tech-Challenge/FtcRobotController/wiki/Blocks-Tutorial)

Even if you are an advanced Java programmer, it is helpful to start with the [FTC Blocks tutorial](https://github.com/FIRST-Tech-Challenge/FtcRobotController/wiki/Blocks-Tutorial), and then migrate to the [OnBot Java Tool](https://github.com/FIRST-Tech-Challenge/FtcRobotController/wiki/OnBot-Java-Tutorial) or to [Android Studio](https://github.com/FIRST-Tech-Challenge/FtcRobotController/wiki/Android-Studio-Tutorial) afterwards.

### Notice!
* Gradle can only execute JDK between 8 and 21. Please choose the correct JDK version to install.
* Remember to choose the correct version based on your platform (Mac, Windows).

## Downloading this project
There are many ways you can use to download this repo:

* If you are a git user, you can clone the most current version of the repository by pasting the below command to your terminal:
<p>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;git clone https://github.com/TeamVietnam/FGC-2022.git</p>

* If you prefer, you can use the "Download Zip" button available through the main repository page. Downloading the project as a .ZIP file will keep the size of the download manageable.

Once you have downloaded and uncompressed (if needed) your folder, you can use Android Studio to import the folder  ("Import project (Eclipse ADT, Gradle, etc.)").

## File hierarchy
There are two main folder that you need to pay attention most to: [FtcRobotController](/FtcRobotController) and [TeamCode](/TeamCode).

The TeamCode folder is the main environment for you to program the robot. The only necessary file to operate the robot is the OpMode file (the inheritance class of class OpMode). 

![Alt](/TeamCode/imglib/OpMode.png "An example of an OpMode file")


## Supported documents
### User Documentation and Tutorials
If you doesn't have a Java background, you can access the following documents to learn:

*FIRST* maintains online documentation with information and tutorials on how to use the *FIRST* Tech Challenge software and robot control system.  You can access this documentation using the following link:

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[FtcRobotController Online Documentation](https://github.com/FIRST-Tech-Challenge/FtcRobotController/wiki)

Note that the online documentation is an "evergreen" document that is constantly being updated and edited.  It contains the most current information about the *FIRST* Tech Challenge software and control system.

### Javadoc Reference Material
The Javadoc reference documentation for the FTC SDK is now available online.  Click on the following link to view the FTC SDK Javadoc documentation as a live website:

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[FTC Javadoc Documentation](https://javadoc.io/doc/org.firstinspires.ftc)

### Online User Forum
For technical questions regarding the Control System or the FTC SDK, please visit the FTC Technology forum:

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[FTC Technology Forum](https://ftcforum.firstinspires.org/forum/ftc-technology)

### Sample OpModes
This project contains a large selection of Sample OpModes (robot code examples) which can be cut and pasted into your /teamcode folder to be used as-is, or modified to suit your team's needs.

Samples Folder: &nbsp;&nbsp; [/FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples](FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples)

The readme.md file located in the [/TeamCode/src/main/java/org/firstinspires/ftc/teamcode](TeamCode/src/main/java/org/firstinspires/ftc/teamcode) folder contains an explanation of the sample naming convention, and instructions on how to copy them to your own project space.


