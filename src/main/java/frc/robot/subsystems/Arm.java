// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.libraries.SparkMaxExtended;

public class Arm extends SubsystemBase {

  private SparkMaxExtended outerArmLeader;
  private SparkMaxExtended outerArmFollower;
  private SparkMaxPIDController outerArmPIDController;
  private RelativeEncoder outerArmEncoder;

  private SparkMaxExtended innerArmLeader;
  private SparkMaxExtended innerArmFollower;
  private SparkMaxPIDController innerArmPIDController;
  private RelativeEncoder innerArmEncoder;

  private SparkMaxExtended wrist;
  private SparkMaxPIDController wristPIDController;
  private RelativeEncoder wristEncoder;

  private DoubleSolenoid mArmSolenoid;
  private boolean mArmIsLocked = true;

  public Arm() {
    initializeOuterArm();
    initializeInnerArm();
    initializeWrist();
    setArmSolenoid(Value.kReverse);
  }

  private void configureSpark(SparkMaxExtended sparkMax, boolean invert) {
    sparkMax.setInverted(invert);
    sparkMax.enableVoltageCompensation(12.0);
    sparkMax.setIdleMode(IdleMode.kBrake);
  }

  public void initializeOuterArm() {
    // Initialize motors
    outerArmLeader = new SparkMaxExtended(Constants.OUTER_ARM_LEADER);
    configureSpark(outerArmLeader, true);

    outerArmFollower = new SparkMaxExtended(Constants.OUTER_ARM_FOLLOWER, outerArmLeader);
    configureSpark(outerArmFollower, true);

    // Initialize PID
    outerArmEncoder = outerArmLeader.getAlternateEncoder(Constants.ALT_ENC_TYPE, Constants.TBE_CPR);
    outerArmEncoder.setPosition(0);

    outerArmPIDController = outerArmLeader.getPIDController();
    outerArmPIDController.setFeedbackDevice(outerArmEncoder);

    // PID Defaults
    outerArmPIDController.setP(0.1);
    outerArmPIDController.setI(0.0);
    outerArmPIDController.setD(0.0);
    outerArmPIDController.setIZone(0.0);
    outerArmPIDController.setFF(0.0);
    // outerArmPIDController.setOutputRange(0, 0);

    SmartDashboard.putNumber("Outer arm position", 0);
  }

  public void initializeInnerArm() {
    // Initialize motors
    innerArmLeader = new SparkMaxExtended(Constants.INNER_ARM_LEADER);
    configureSpark(innerArmLeader, true);

    innerArmFollower = new SparkMaxExtended(Constants.INNER_ARM_FOLLOWER, innerArmLeader);
    configureSpark(innerArmFollower, true);

    // Initialize PID
    innerArmEncoder = innerArmLeader.getAlternateEncoder(Constants.ALT_ENC_TYPE, Constants.TBE_CPR);
    innerArmPIDController = innerArmLeader.getPIDController();
    innerArmPIDController.setFeedbackDevice(innerArmEncoder);

    // PID Defaults
    innerArmPIDController.setP(0.1);
    innerArmPIDController.setI(0.0);
    innerArmPIDController.setD(0.0);
    innerArmPIDController.setIZone(0.0);
    innerArmPIDController.setFF(0.0);
    // innerArmPIDController.setOutputRange(0, 0);
  }

  public void initializeWrist() {
    // Initialize motors
    wrist = new SparkMaxExtended(Constants.WRIST_MOTOR);
    configureSpark(wrist, true);

    // Initialize PID
    wristEncoder = wrist.getAlternateEncoder(Constants.ALT_ENC_TYPE, Constants.TBE_CPR);
    wristPIDController = wrist.getPIDController();
    wristPIDController.setFeedbackDevice(wristEncoder);

    // PID Defaults
    wristPIDController.setP(0.1);
    wristPIDController.setI(0.0);
    wristPIDController.setD(0.0);
    wristPIDController.setIZone(0.0);
    wristPIDController.setFF(0.0);
    // wristPIDController.setOutputRange(0, 0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Arm locked", mArmIsLocked);

    double oap = SmartDashboard.getNumber("Outer arm position", 0);
    outerArmPIDController.setReference(oap, CANSparkMax.ControlType.kPosition);
  }

  public void setArmSolenoid(Value solenoidPosition) {
    mArmIsLocked = (solenoidPosition == DoubleSolenoid.Value.kForward);
    mArmSolenoid.set(solenoidPosition);
  }
}