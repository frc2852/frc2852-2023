// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.libraries.SparkMaxExtended;

public class Arm extends SubsystemBase {

  private SparkMaxExtended outerArmLeader = null;
  private SparkMaxExtended outerArmFollower = null;
  private SparkMaxExtended innerArmLeader = null;
  private SparkMaxExtended innerArmFollower = null;
  private SparkMaxExtended wrist = null;

  private DoubleSolenoid mArmSolenoid;
  private boolean mArmIsLocked = true;

  public Arm() {
    // initializeOuterArm();
    // initializeInnerArm();
    initializeWrist();
    // setArmSolenoid(Value.kReverse);
  }

  private void configureSpark(SparkMaxExtended sparkMax, boolean invert) {
    sparkMax.setInverted(invert);
    sparkMax.enableVoltageCompensation(12.0);
    // sparkMax.setClosedLoopRampRate(Constants.DRIVE_VOLTAGE_RAMP_RATE);
    sparkMax.setIdleMode(IdleMode.kBrake);
  }

  public void initializeOuterArm() {
    outerArmLeader = new SparkMaxExtended(Constants.OUTER_ARM_LEADER);
    configureSpark(outerArmLeader, true);

    outerArmFollower = new SparkMaxExtended(Constants.OUTER_ARM_FOLLOWER, outerArmLeader);
    configureSpark(outerArmFollower, true);

    // 3. ... (auto position, if no auto teleop position)
  }

  public void initializeInnerArm() {
    innerArmLeader = new SparkMaxExtended(Constants.INNER_ARM_LEADER);
    configureSpark(innerArmLeader, true);

    innerArmFollower = new SparkMaxExtended(Constants.INNER_ARM_FOLLOWER, innerArmLeader);
    configureSpark(innerArmFollower, true);

    // 3. ... (auto position, if no auto teleop position)
  }

  public void initializeWrist() {
    wrist = new SparkMaxExtended(55);
    configureSpark(wrist, false);

    // 2. ... (auto position, if no auto teleop position)
  }

  @Override
  public void periodic() {
  }

  public void OuterArmMovement(int position) {
    outerArmLeader.set(ControlType.kPosition, position);
  }

  public void setArmSolenoid(Value solenoidPosition) {
    mArmIsLocked = (solenoidPosition == DoubleSolenoid.Value.kForward);
    mArmSolenoid.set(solenoidPosition);
  }
}
