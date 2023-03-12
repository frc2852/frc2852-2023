// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {

  private static final SparkMaxAlternateEncoder.Type kAltEncType = SparkMaxAlternateEncoder.Type.kQuadrature;
  private static final int kCPR = 8192;

  // #region Outer Arm

  private CANSparkMax outerArmLeader;
  private CANSparkMax outerArmFollower;
  private SparkMaxPIDController outerArmPIDController;
  private RelativeEncoder outerArmEncoder;

  public double outerP, outerI, outerD, outerIz, outerFF, outerMax, outerMin;

  // #endregion

  // #region Inner Arm

  private CANSparkMax innerArmLeader;
  private CANSparkMax innerArmFollower;
  private SparkMaxPIDController innerArmPIDController;
  private RelativeEncoder innerArmEncoder;

  public double innerP, innerI, innerD, innerMax, innerMin;

  // #endregion

  // #region Wrist

  private CANSparkMax wrist;
  private SparkMaxPIDController wristPIDController;
  private RelativeEncoder wristEncoder;

  public double wristP, wristI, wristD, wristMax, wristMin;

  // #endregion

  private DoubleSolenoid mArmSolenoid;
  private boolean mArmIsLocked = true;

  public ArmSubsystem() {
    // InitializeOuterArm();
    // InitializeInnerArm();
    InitializeWrist();

    mArmSolenoid = new DoubleSolenoid(Constants.PNEUMATIC_HUB, PneumaticsModuleType.REVPH, Constants.ARM_UNLOCKED, Constants.ARM_LOCKED);
    setArmSolenoid(Value.kReverse);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Arm Locked", mArmIsLocked);

    OuterArmPeriodic();
    InnerArmPeriodic();
    WristPeriodic();
  }

  private void InitializeOuterArm() {
    // Spark Max
    outerArmLeader = new CANSparkMax(Constants.OUTER_ARM_LEADER, MotorType.kBrushless);
    outerArmLeader.restoreFactoryDefaults();
    outerArmLeader.setIdleMode(IdleMode.kBrake);
    // outerArmLeader.enableVoltageCompensation(12.0);

    outerArmFollower = new CANSparkMax(Constants.OUTER_ARM_FOLLOWER, MotorType.kBrushless);
    outerArmFollower.restoreFactoryDefaults();
    outerArmFollower.setIdleMode(IdleMode.kBrake);
    // outerArmFollower.enableVoltageCompensation(12.0);
    outerArmFollower.follow(outerArmLeader);

    // Encoder
    outerArmEncoder = outerArmLeader.getEncoder(); //outerArmLeader.getAlternateEncoder(kAltEncType, kCPR);
    // outerArmEncoder.setInverted(true);
    outerArmEncoder.setPosition(0);

    // PID Controller
    outerArmPIDController = outerArmLeader.getPIDController();
    // outerArmPIDController.setFeedbackDevice(outerArmEncoder);

    // PID defaults
    outerP = 0.1;
    outerI = 0;
    outerD = 0;
    outerIz = 0;
    outerFF = 0;
    outerMax = 0.1;
    outerMin = -0.1;

    // set PID coefficients
    outerArmPIDController.setP(outerP);
    outerArmPIDController.setI(outerI);
    outerArmPIDController.setD(outerD);
    outerArmPIDController.setIZone(outerIz);
    outerArmPIDController.setFF(outerFF);
    outerArmPIDController.setOutputRange(outerMin, outerMax);

    // Display PID
    SmartDashboard.putNumber("OuterP", outerP);
    SmartDashboard.putNumber("OuterI", outerI);
    SmartDashboard.putNumber("OuterD", outerD);
    SmartDashboard.putNumber("OuterMax", outerMax);
    SmartDashboard.putNumber("OuterMin", outerMin);
    SmartDashboard.putNumber("OuterIz", outerMax);
    SmartDashboard.putNumber("OuterFF", outerMin);
    SmartDashboard.putNumber("OuterRotation", 0);
  }

  private void InitializeInnerArm() {
    // Spark Max
    innerArmLeader = new CANSparkMax(Constants.INNER_ARM_LEADER, MotorType.kBrushless);
    innerArmLeader.restoreFactoryDefaults();
    innerArmLeader.setIdleMode(IdleMode.kBrake);
    innerArmLeader.enableVoltageCompensation(12.0);

    innerArmFollower = new CANSparkMax(Constants.INNER_ARM_FOLLOWER, MotorType.kBrushless);
    innerArmFollower.restoreFactoryDefaults();
    innerArmFollower.setIdleMode(IdleMode.kBrake);
    innerArmFollower.enableVoltageCompensation(12.0);
    innerArmFollower.follow(innerArmLeader);

    // Encoder
    innerArmEncoder = innerArmLeader.getAlternateEncoder(kAltEncType, kCPR);
    innerArmEncoder.setInverted(false);
    innerArmEncoder.setPosition(0);

    // PID Controller
    innerArmPIDController = innerArmLeader.getPIDController();
    innerArmPIDController.setFeedbackDevice(innerArmEncoder);

    // PID defaults
    innerP = 0.1;
    innerI = 0;
    innerD = 0;
    innerMax = 0.1;
    innerMin = -0.1;

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
    wrist = new CANSparkMax(Constants.OUTER_ARM_LEADER, MotorType.kBrushless);
    wrist.restoreFactoryDefaults();
    wrist.setIdleMode(IdleMode.kBrake);
    wrist.enableVoltageCompensation(12.0);

    // PID Controller
    wristPIDController = wrist.getPIDController();

    // Encoder
    wristEncoder = wrist.getEncoder();
    wristEncoder.setPosition(0);

    // PID defaults
    wristP = 0.1;
    wristI = 0;
    wristD = 0;
    wristMax = 0.1;
    wristMin = -0.1;

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

    outerArmPIDController.setReference(rotations, CANSparkMax.ControlType.kPosition);

    SmartDashboard.putNumber("OuterSetPoint", rotations);
    SmartDashboard.putNumber("OuterPosition", outerArmEncoder.getPosition());
  }

  private void InnerArmPeriodic() {
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

    innerArmPIDController.setReference(rotations, CANSparkMax.ControlType.kPosition);

    SmartDashboard.putNumber("InnerSetPoint", rotations);
    SmartDashboard.putNumber("InnerPosition", innerArmEncoder.getPosition());
  }

  private void WristPeriodic() {
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

    wristPIDController.setReference(rotations, CANSparkMax.ControlType.kPosition);

    SmartDashboard.putNumber("WristSetPoint", rotations);
    SmartDashboard.putNumber("WristPosition", wristEncoder.getPosition());
  }

  private void setArmSolenoid(Value solenoidPosition) {
    mArmIsLocked = (solenoidPosition == DoubleSolenoid.Value.kForward);
    mArmSolenoid.set(solenoidPosition);
  }
}