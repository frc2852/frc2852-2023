// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.fasterxml.jackson.databind.deser.std.StdScalarDeserializer;
import com.revrobotics.SparkMaxAlternateEncoder;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  // Controller
  public static final double DEAD_ZONE = 0.05;

  // Controller mapping
  public static final int DRIVER_CONTROLLER = 0;
  public static final int OPERATOR_CONTROLLER = 1;

  // Joystick Mapping
  public static final int LEFT_STICK_VERT = 1;
  public static final int LEFT_STICK_HORZ = 0;
  public static final int RIGHT_STICK_VERT = 5;
  public static final int RIGHT_STICK_HORZ = 4;

  // Trigger Mapping
  public static int LEFT_TRIGGER = 2;
  public static int RIGHT_TRIGGER = 3;

  // Drive
  public static final int DRIVE_LEFT_LEADER = 1;
  public static final int DRIVE_LEFT_FOLLOWER = 2;

  public static final int DRIVE_RIGHT_LEADER = 3;
  public static final int DRIVE_RIGHT_FOLLOWER = 4;

  // Intake
  public static final int INTAKE_LEFT_BOTTOM = 6;
  public static final int INTAKE_RIGHT_BOTTOM = 25;

  public static final int INTAKE_OPEN = 3;
  public static final int INTAKE_CLOSE = 12;

  // Arm
  public static final int OUTER_ARM_LEADER = 9;
  public static final int OUTER_ARM_FOLLOWER = 8;

  public static final int INNER_ARM_LEADER = 10;
  public static final int INNER_ARM_FOLLOWER = 11;

  public static final int WRIST_MOTOR = 12;

  public static final int ARM_UNLOCKED = 7;
  public static final int ARM_LOCKED = 8;

  // Gearbox
  public static final int DRIVE_GEAR_BOX_OPEN = 5;
  public static final int DRIVE_GEAR_BOX_CLOSE = 10;

  // Voltage
  public static final double DRIVE_VOLTAGE_RAMP_RATE = 0;

  // PneumaticHub
  public static final int PNEUMATIC_HUB = 13;

  public static final int PDP = 14;
  public static final int PIGEON_IMU = 15;

  // Limit Switches
  public static final int BOTTOM_INTAKE_LIMIT_SWITCH = 0;
}
