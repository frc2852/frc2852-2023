Team 2852 FRC - 2023 Code
=========================

Build Status
------------

[![Build Status](https://github.com/frc2852/frc2852-2023/actions/workflows/main.yml/badge.svg)](https://github.com/frc2852/frc2852-2023/actions)

Overview
--------

This repository contains the codebase for Team 2852 FRC's 2023 robot. The code is written in Java using WPILib 2023 and runs on a roboRIO.

The robot has the following subsystems:

-   DriveSubsystem: West coast drive controlled only by the left stick on the driver Xbox controller.
-   ArmSubsystem: Three pivot arm with position control set by the driver and operator using the A,B,X,Y buttons. Manual control of the arms can be done by the D-Pad on the operator Xbox controller.
-   IntakeSubsystem: Controls a piston and two intake motors.

All motors are NEOs using Spark Max controllers.

Installation
------------

To use this code, clone the repository onto your local machine:

bashCopy code

`git clone https://github.com/team2852/2023-code.git`

Then, open the project in Visual Studio Code with WPILib 2023 and deploy to the robot.

Usage
-----

The robot can be controlled using Xbox controllers. The left stick on the driver controller controls the west coast drive.

The arm positions are set by the driver and operator using the A,B,X,Y buttons. The D-Pad on the operator controller can be used for manual control of the arms.

In autonomous mode, there are multiple modes which can be selected on the SmartDashboard.

License
-------

This code is licensed under the MIT License. See the [LICENSE](https://chat.openai.com/LICENSE) file for details.
