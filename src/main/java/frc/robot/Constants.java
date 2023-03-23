// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
  public static final int PIGEON_IMU = 15;

  public static final double WHEEL_DIAMETER_INCHES = 6.0;
  public static final double GEAR_RATIO = 15.0;
  public static final double COUNTS_PER_REVOLUTION = 8192.0; // TODO: Change to 42 if using the hall sensor
  public static final double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER_INCHES;
  public static final double COUNTS_PER_WHEEL_REVOLUTION = COUNTS_PER_REVOLUTION * GEAR_RATIO;
  public static final double DRIVE_ENCODER_CONVERSION_FACTOR = COUNTS_PER_WHEEL_REVOLUTION / WHEEL_CIRCUMFERENCE;

  public static final int ONE_FOOT = 12;
  public enum ArmPosition{
    ZERO,
    PICK_UP,
    HIGH_PICK_UP,
    DRIVE,
    LOW_GOAL,
    MID_GOAL,
    HIGH_GOAL
  } 
}
