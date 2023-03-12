// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {

  private CANSparkMax outerArmLeader;
  private CANSparkMax outerArmFollower;
  private SparkMaxPIDController outerArmPIDController;
  private RelativeEncoder outerArmEncoder;
  public double outerP, outerI, outerD, outerMax, outerMin;

  private CANSparkMax innerArmLeader;
  private CANSparkMax innerArmFollower;
  private SparkMaxPIDController innerArmPIDController;
  private RelativeEncoder innerArmEncoder;
  public double innerP, innerI, innerD, innerMax, innerMin;

  private CANSparkMax wrist;
  private SparkMaxPIDController wristPIDController;
  private RelativeEncoder wristEncoder;
  public double wristP, wristI, wristD, wristMax, wristMin;

  private DoubleSolenoid mArmSolenoid;
  private boolean mArmIsLocked = true;

  // This should only be enabled for testing and tuning.
  // Automated positions will not work when enabled
  private final boolean ARM_DEBUG = true;

  private static double OUTER_ARM_POSITION = 0;
  private static double INNER_ARM_POSITION = 0;
  private static double WRIST_POSITION = 0;

  private static double ERROR_RANGE = 1;

  private final double OUTER_ARM_MAX_SPEED = 0.3;
  private final double INNER_ARM_MAX_SPEED = 0.2;
  private final double WRIST_MAX_SPEED = 0.2;

  public ArmSubsystem() {
    InitializeOuterArm();
    InitializeInnerArm();
    InitializeWrist();

    mArmSolenoid = new DoubleSolenoid(Constants.PNEUMATIC_HUB, PneumaticsModuleType.REVPH, Constants.ARM_UNLOCKED,
        Constants.ARM_LOCKED);
    setArmSolenoid(Value.kReverse);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Arm Locked", mArmIsLocked);

    OuterArmPeriodic();
    InnerArmPeriodic();
    WristPeriodic();

    if (ARM_DEBUG) {
      OuterArmDebugPeriodic();
      InnerArmDebugPeriodic();
      WristDebugPeriodic();
    }
  }

  public void SetOuterArmPosition(double position, double speed) {
    if (speed == 0) {
      outerArmPIDController.setOutputRange(-OUTER_ARM_MAX_SPEED, OUTER_ARM_MAX_SPEED);
    } else {
      outerArmPIDController.setOutputRange(-speed, speed);
    }

    OUTER_ARM_POSITION = position;
  }

  public boolean IsOuterArmAtPosition() {
    return (Math.abs(OUTER_ARM_POSITION - outerArmEncoder.getPosition()) <= ERROR_RANGE);
  }

  public void SetInnerArmPosition(double position, double speed) {
    if (speed == 0) {
      innerArmPIDController.setOutputRange(-INNER_ARM_MAX_SPEED, INNER_ARM_MAX_SPEED);
    } else {
      innerArmPIDController.setOutputRange(-speed, speed);
    }

    INNER_ARM_POSITION = position;
  }

  public boolean IsInnerArmAtPosition() {
    return (Math.abs(INNER_ARM_POSITION - innerArmEncoder.getPosition()) <= ERROR_RANGE);
  }

  public void SetWristPosition(double position, double speed) {
    if (speed == 0) {
      wristPIDController.setOutputRange(-WRIST_MAX_SPEED, WRIST_MAX_SPEED);
    } else {
      wristPIDController.setOutputRange(-speed, speed);
    }

    WRIST_POSITION = position;
  }

  public boolean IsWristAtPosition() {
    return (Math.abs(WRIST_POSITION - wristEncoder.getPosition()) <= ERROR_RANGE);
  }

  private void InitializeOuterArm() {
    // Spark Max
    outerArmLeader = new CANSparkMax(Constants.OUTER_ARM_LEADER, MotorType.kBrushless);
    outerArmLeader.restoreFactoryDefaults();
    outerArmLeader.setIdleMode(IdleMode.kBrake);

    outerArmFollower = new CANSparkMax(Constants.OUTER_ARM_FOLLOWER, MotorType.kBrushless);
    outerArmFollower.restoreFactoryDefaults();
    outerArmFollower.setIdleMode(IdleMode.kBrake);
    outerArmFollower.follow(outerArmLeader);

    // PID Controller
    outerArmPIDController = outerArmLeader.getPIDController();

    // Encoder
    outerArmEncoder = outerArmLeader.getEncoder();
    outerArmEncoder.setPosition(0);

    // PID defaults
    outerP = 0.1;
    outerI = 0.0000000000001;
    outerD = 1.0;
    outerMax = OUTER_ARM_MAX_SPEED;
    outerMin = -OUTER_ARM_MAX_SPEED;

    // set PID coefficients
    outerArmPIDController.setP(outerP);
    outerArmPIDController.setI(outerI);
    outerArmPIDController.setD(outerD);
    outerArmPIDController.setIZone(0);
    outerArmPIDController.setFF(0);
    outerArmPIDController.setOutputRange(outerMin, outerMax);

    // Display PID
    SmartDashboard.putNumber("OuterP", outerP);
    SmartDashboard.putNumber("OuterI", outerI);
    SmartDashboard.putNumber("OuterD", outerD);
    SmartDashboard.putNumber("OuterMax", outerMax);
    SmartDashboard.putNumber("OuterMin", outerMin);
    SmartDashboard.putNumber("OuterRotation", 0);
  }

  private void InitializeInnerArm() {
    // Spark Max
    innerArmLeader = new CANSparkMax(Constants.INNER_ARM_LEADER, MotorType.kBrushless);
    innerArmLeader.restoreFactoryDefaults();
    innerArmLeader.setInverted(true);
    innerArmLeader.setIdleMode(IdleMode.kBrake);

    innerArmFollower = new CANSparkMax(Constants.INNER_ARM_FOLLOWER, MotorType.kBrushless);
    innerArmFollower.restoreFactoryDefaults();
    innerArmFollower.follow(innerArmLeader, true);
    innerArmFollower.setIdleMode(IdleMode.kBrake);

    // PID Controller
    innerArmPIDController = innerArmLeader.getPIDController();

    // Encoder
    innerArmEncoder = innerArmLeader.getEncoder();
    innerArmEncoder.setPosition(0);

    // PID defaults
    innerP = 0.3;
    innerI = 0.0000000003;
    innerD = 1.0;
    innerMax = INNER_ARM_MAX_SPEED;
    innerMin = -INNER_ARM_MAX_SPEED;

    // set PID coefficients
    innerArmPIDController.setP(innerP);
    innerArmPIDController.setI(innerI);
    innerArmPIDController.setD(innerD);
    innerArmPIDController.setIZone(0);
    innerArmPIDController.setFF(0);
    innerArmPIDController.setOutputRange(innerMin, innerMax);

    // Display PID
    SmartDashboard.putNumber("InnerP", innerP);
    SmartDashboard.putNumber("InnerI", innerI);
    SmartDashboard.putNumber("InnerD", innerD);
    SmartDashboard.putNumber("InnerMax", innerMax);
    SmartDashboard.putNumber("InnerMin", innerMin);
    SmartDashboard.putNumber("InnerRotation", 0);
  }

  private void InitializeWrist() {
    // Spark Max
    wrist = new CANSparkMax(Constants.WRIST_MOTOR, MotorType.kBrushless);
    wrist.restoreFactoryDefaults();
    wrist.setIdleMode(IdleMode.kBrake);
    wrist.setInverted(true);

    // PID Controller
    wristPIDController = wrist.getPIDController();

    // Encoder
    wristEncoder = wrist.getEncoder();
    wristEncoder.setPosition(0);

    // PID defaults
    wristP = 0.3;
    wristI = 0.0000003;
    wristD = 1.0;
    wristMax = WRIST_MAX_SPEED;
    wristMin = -WRIST_MAX_SPEED;

    // set PID coefficients
    wristPIDController.setP(wristP);
    wristPIDController.setI(wristI);
    wristPIDController.setD(wristD);
    wristPIDController.setIZone(0);
    wristPIDController.setFF(0);
    wristPIDController.setOutputRange(wristMin, wristMax);

    // Display PID
    SmartDashboard.putNumber("WristP", wristP);
    SmartDashboard.putNumber("WristI", wristI);
    SmartDashboard.putNumber("WristD", wristD);
    SmartDashboard.putNumber("WristMax", wristMax);
    SmartDashboard.putNumber("WristMin", wristMin);
    SmartDashboard.putNumber("WristRotation", 0);
  }

  private void OuterArmPeriodic() {
    outerArmPIDController.setReference(OUTER_ARM_POSITION, CANSparkMax.ControlType.kPosition);
    SmartDashboard.putNumber("OuterSetPoint", OUTER_ARM_POSITION);
    SmartDashboard.putNumber("OuterPosition", outerArmEncoder.getPosition());
    SmartDashboard.putBoolean("Outer Arm Position", IsOuterArmAtPosition());
  }

  private void InnerArmPeriodic() {
    innerArmPIDController.setReference(INNER_ARM_POSITION, CANSparkMax.ControlType.kPosition);
    SmartDashboard.putNumber("InnerSetPoint", INNER_ARM_POSITION);
    SmartDashboard.putNumber("InnerPosition", innerArmEncoder.getPosition());
    SmartDashboard.putBoolean("Inner Arm Position", IsInnerArmAtPosition());
  }

  private void WristPeriodic() {
    wristPIDController.setReference(WRIST_POSITION, CANSparkMax.ControlType.kPosition);
    SmartDashboard.putNumber("WristSetPoint", WRIST_POSITION);
    SmartDashboard.putNumber("WristPosition", wristEncoder.getPosition());
    SmartDashboard.putBoolean("wrist Arm Position", IsWristAtPosition());
  }

  /**
   * Outer Arm periodic is for testing and tuning only and should not be used or
   * called for automated code.
   * By default this function is disabled unless the constant 'ARM_DEBUG' is set
   * to true
   */
  private void OuterArmDebugPeriodic() {
    // Read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("OuterP", 0);
    double i = SmartDashboard.getNumber("OuterI", 0);
    double d = SmartDashboard.getNumber("OuterD", 0);
    double max = SmartDashboard.getNumber("OuterMax", 0);
    double min = SmartDashboard.getNumber("OuterMin", 0);

    double rotations = SmartDashboard.getNumber("OuterRotation", 0);

    // Update values
    if ((p != outerP)) {
      outerArmPIDController.setP(p);
      outerP = p;
    }
    if ((i != outerI)) {
      outerArmPIDController.setI(i);
      outerI = i;
    }
    if ((d != outerD)) {
      outerArmPIDController.setD(d);
      outerD = d;
    }

    if ((max != outerMax) || (min != outerMin)) {
      outerArmPIDController.setOutputRange(min, max);
      outerMin = min;
      outerMax = max;
    }

    SetOuterArmPosition(rotations, 0);
  }

  /**
   * Inner Arm periodic is for testing and tuning only and should not be used or
   * called for automated code.
   * By default this function is disabled unless the constant 'ARM_DEBUG' is set
   * to true
   */
  private void InnerArmDebugPeriodic() {
    // Read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("InnerP", 0);
    double i = SmartDashboard.getNumber("InnerI", 0);
    double d = SmartDashboard.getNumber("InnerD", 0);
    double max = SmartDashboard.getNumber("InnerMax", 0);
    double min = SmartDashboard.getNumber("InnerMin", 0);

    double rotations = SmartDashboard.getNumber("InnerRotation", 0);

    // Update values
    if ((p != innerP)) {
      innerArmPIDController.setP(p);
      innerP = p;
    }
    if ((i != innerI)) {
      innerArmPIDController.setI(i);
      innerI = i;
    }
    if ((d != innerD)) {
      innerArmPIDController.setD(d);
      innerD = d;
    }

    if ((max != innerMax) || (min != innerMin)) {
      innerArmPIDController.setOutputRange(min, max);
      innerMin = min;
      innerMax = max;
    }

    SetInnerArmPosition(rotations, 0);
  }

  /**
   * Wrist periodic is for testing and tuning only and should not be used or
   * called for automated code.
   * By default this function is disabled unless the constant 'ARM_DEBUG' is set
   * to true
   */
  private void WristDebugPeriodic() {
    // Read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("WristP", 0);
    double i = SmartDashboard.getNumber("WristI", 0);
    double d = SmartDashboard.getNumber("WristD", 0);
    double max = SmartDashboard.getNumber("WristMax", 0);
    double min = SmartDashboard.getNumber("WristMin", 0);

    double rotations = SmartDashboard.getNumber("WristRotation", 0);

    // Update values
    if ((p != wristP)) {
      wristPIDController.setP(p);
      wristP = p;
    }
    if ((i != wristI)) {
      wristPIDController.setI(i);
      wristI = i;
    }
    if ((d != wristD)) {
      wristPIDController.setD(d);
      wristD = d;
    }

    if ((max != wristMax) || (min != wristMin)) {
      wristPIDController.setOutputRange(min, max);
      wristMin = min;
      wristMax = max;
    }

    SetWristPosition(rotations, 0);
  }

  private void setArmSolenoid(Value solenoidPosition) {
    mArmIsLocked = (solenoidPosition == DoubleSolenoid.Value.kForward);
    mArmSolenoid.set(solenoidPosition);
  }
}