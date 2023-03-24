// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmPosition;

public class ArmSubsystem extends SubsystemBase {

  // Hardware
  private CANSparkMax outerArmLeader, outerArmFollower;
  private SparkMaxPIDController outerArmPIDController;
  private RelativeEncoder outerArmEncoder;

  private CANSparkMax innerArmLeader, innerArmFollower;
  private SparkMaxPIDController innerArmPIDController;
  private RelativeEncoder innerArmEncoder;

  private CANSparkMax wrist;
  private SparkMaxPIDController wristPIDController;
  private RelativeEncoder wristEncoder;

  private DoubleSolenoid mArmSolenoid;
  private boolean mArmIsLocked = true;

  // Arm positioning
  public static ArmPosition ARM_POSITION = ArmPosition.DRIVE;
  public static double OUTER_ARM_POSITION = 0;
  public static double INNER_ARM_POSITION = 0;
  public static double WRIST_POSITION = 0;

  // Constants
  private final double OUTER_ARM_MAX_SPEED = 0.3;
  private final double OUTER_ARM_SMARTMOTION_MAXVEL = 2000;
  private final double OUTER_ARM_SMARTMOTION_MAXACC = 1500;

  private final double INNER_ARM_MAX_SPEED = 0.2;
  private final double INNER_ARM_SMARTMOTION_MAXVEL = 2000;
  private final double INNER_ARM_SMARTMOTION_MAXACC = 1500;

  private final double WRIST_MAX_SPEED = 0.2;
  private final double WRIST_SMARTMOTION_MAXVEL = 2000;
  private final double WRIST_SMARTMOTION_MAXACC = 1500;

  private final double ERROR_RANGE = 0.1;
  private final double TOLERANCE = 1e-2;

  private final double HALL_ENCODER_COUNT = 42; // 8192;
  private final double TBE_ENCODER_COUNT = 8192;

  // This should only be enabled for testing and tuning.
  // Automated positions will not work when enabled
  private boolean ARM_DEBUG = false;

  // GET
  public boolean isOuterArmAtPosition() {
    return (Math.abs(OUTER_ARM_POSITION - outerArmEncoder.getPosition()) <= TOLERANCE);
  }

  public boolean isInnerArmAtPosition() {
    return (Math.abs(INNER_ARM_POSITION - innerArmEncoder.getPosition()) <= TOLERANCE);
  }

  public boolean isWristAtPosition() {
    return (Math.abs(WRIST_POSITION - wristEncoder.getPosition()) <= TOLERANCE);
  }

  // SET
  // TODO: After validating smart motion, remove the pivotSpeed from here.
  public void setOuterArmPosition(double position, double pivotSpeed) {
    if (pivotSpeed == 0) {
      outerArmPIDController.setOutputRange(-OUTER_ARM_MAX_SPEED, OUTER_ARM_MAX_SPEED);
    } else {
      outerArmPIDController.setOutputRange(-pivotSpeed, pivotSpeed);
    }

    OUTER_ARM_POSITION = position;
  }

  public void setInnerArmPosition(double position, double pivotSpeed) {
    if (pivotSpeed == 0) {
      innerArmPIDController.setOutputRange(-INNER_ARM_MAX_SPEED, INNER_ARM_MAX_SPEED);
    } else {
      innerArmPIDController.setOutputRange(-pivotSpeed, pivotSpeed);
    }

    INNER_ARM_POSITION = position;
  }

  public void setWristPosition(double position, double pivotSpeed) {
    if (pivotSpeed == 0) {
      wristPIDController.setOutputRange(-WRIST_MAX_SPEED, WRIST_MAX_SPEED);
    } else {
      wristPIDController.setOutputRange(-pivotSpeed, pivotSpeed);
    }

    WRIST_POSITION = position;
  }

  public ArmSubsystem() {
    // DO NOT REMOVE OR MODIFY THIS CODE
    if (DriverStation.isFMSAttached()) {
      ARM_DEBUG = false;
    }
    // END

    initOuterArm();
    initInnerArm();
    initWrist();

    mArmSolenoid = new DoubleSolenoid(Constants.PNEUMATIC_HUB, PneumaticsModuleType.REVPH, Constants.ARM_UNLOCKED,
        Constants.ARM_LOCKED);
    setArmSolenoid(Value.kReverse);
  }

  private void initOuterArm() {
    outerArmLeader = initMotor(Constants.OUTER_ARM_LEADER, false, null);
    outerArmFollower = initMotor(Constants.OUTER_ARM_FOLLOWER, false, outerArmLeader);
    outerArmEncoder = initEncoder(outerArmLeader, 229.5 * HALL_ENCODER_COUNT); // TODO: Change to through bore encoder

    // PID Controller
    outerArmPIDController = outerArmLeader.getPIDController();
    initPIDController(
        outerArmPIDController,
        "Outer",
        0.1,
        0.0000000000001,
        1.0,
        0,
        -OUTER_ARM_MAX_SPEED,
        OUTER_ARM_MAX_SPEED,
        OUTER_ARM_SMARTMOTION_MAXVEL,
        OUTER_ARM_SMARTMOTION_MAXACC,
        ERROR_RANGE);
  }

  private void initInnerArm() {
    innerArmLeader = initMotor(Constants.INNER_ARM_LEADER, true, null);
    innerArmFollower = initMotor(Constants.INNER_ARM_FOLLOWER, true, innerArmLeader);
    innerArmEncoder = initEncoder(innerArmLeader, 60 * HALL_ENCODER_COUNT); // TODO: Change to through bore encoder

    // PID Controller
    innerArmPIDController = innerArmLeader.getPIDController();
    initPIDController(
        innerArmPIDController,
        "Inner",
        0.1,
        0.0000000001,
        1.0,
        0,
        -INNER_ARM_MAX_SPEED,
        INNER_ARM_MAX_SPEED,
        INNER_ARM_SMARTMOTION_MAXVEL,
        INNER_ARM_SMARTMOTION_MAXACC,
        ERROR_RANGE);
  }

  private void initWrist() {
    wrist = initMotor(Constants.WRIST_MOTOR, true, null);
    wristEncoder = initEncoder(wrist, 49 * HALL_ENCODER_COUNT);

    // PID Controller
    wristPIDController = wrist.getPIDController();
    initPIDController(
        wristPIDController,
        "Wrist",
        0.3,
        0.0000003,
        1.0,
        0,
        -WRIST_MAX_SPEED,
        WRIST_MAX_SPEED,
        WRIST_SMARTMOTION_MAXVEL,
        WRIST_SMARTMOTION_MAXACC,
        ERROR_RANGE);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Arm Locked", mArmIsLocked);

    OuterArmPeriodic();
    InnerArmPeriodic();
    WristPeriodic();

    if (ARM_DEBUG) {
      updateAndSetPidValues("Outer", outerArmPIDController, outerArmEncoder);
      updateAndSetPidValues("Inner", innerArmPIDController, innerArmEncoder);
      updateAndSetPidValues("Wrist", wristPIDController, wristEncoder);
    }
  }

  private void OuterArmPeriodic() {
    outerArmPIDController.setReference(OUTER_ARM_POSITION, CANSparkMax.ControlType.kSmartMotion);
    SmartDashboard.putNumber("OuterSetPoint", OUTER_ARM_POSITION);
    SmartDashboard.putNumber("OuterPosition", outerArmEncoder.getPosition());
    SmartDashboard.putBoolean("Outer Arm Position", isOuterArmAtPosition());
  }

  private void InnerArmPeriodic() {
    innerArmPIDController.setReference(INNER_ARM_POSITION, CANSparkMax.ControlType.kSmartMotion);
    SmartDashboard.putNumber("InnerSetPoint", INNER_ARM_POSITION);
    SmartDashboard.putNumber("InnerPosition", innerArmEncoder.getPosition());
    SmartDashboard.putBoolean("Inner Arm Position", isInnerArmAtPosition());
  }

  private void WristPeriodic() {
    wristPIDController.setReference(WRIST_POSITION, CANSparkMax.ControlType.kSmartMotion);
    SmartDashboard.putNumber("WristSetPoint", WRIST_POSITION);
    SmartDashboard.putNumber("WristPosition", wristEncoder.getPosition());
    SmartDashboard.putBoolean("wrist Arm Position", isWristAtPosition());
  }

  private void updateAndSetPidValues(String armName, SparkMaxPIDController armPIDController, RelativeEncoder encoder) {
    // Read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber(armName + "P", 0);
    double i = SmartDashboard.getNumber(armName + "I", 0);
    double d = SmartDashboard.getNumber(armName + "D", 0);
    double max = SmartDashboard.getNumber(armName + "Max", 0);
    double min = SmartDashboard.getNumber(armName + "Min", 0);
    double ff = SmartDashboard.getNumber(armName + "FF", 0);

    double rotations = SmartDashboard.getNumber(armName + "Rotation", 0);

    // Update values
    if ((p != armPIDController.getP())) {
      armPIDController.setP(p);
    }
    if ((i != armPIDController.getI())) {
      armPIDController.setI(i);
    }
    if ((d != armPIDController.getD())) {
      armPIDController.setD(d);
    }
    if ((max != armPIDController.getOutputMax()) || (min != armPIDController.getOutputMin())) {
      armPIDController.setOutputRange(min, max);
    }
    if ((ff != armPIDController.getFF())) {
      armPIDController.setFF(ff);
    }

    switch (armName) {
      case "Outer":
        setOuterArmPosition(rotations, 0);
        break;
      case "Inner":
        setInnerArmPosition(rotations, 0);
        break;
      case "Wrist":
        setWristPosition(rotations, 0);
        break;
    }
  }

  private void setArmSolenoid(Value solenoidPosition) {
    mArmIsLocked = (solenoidPosition == DoubleSolenoid.Value.kForward);
    mArmSolenoid.set(solenoidPosition);
  }

  private CANSparkMax initMotor(int deviceId, boolean isInverted, CANSparkMax leader) {
    CANSparkMax motor = new CANSparkMax(deviceId, MotorType.kBrushless);
    motor.restoreFactoryDefaults();
    motor.setInverted(isInverted);
    motor.enableVoltageCompensation(12.0);
    motor.setSmartCurrentLimit(40);
    motor.setIdleMode(IdleMode.kBrake);

    if (leader != null) {
      motor.follow(leader);
      setPeriodicFramePeriods(motor);
    }

    motor.burnFlash();
    return motor;
  }

  private RelativeEncoder initEncoder(CANSparkMax motor, double conversionFactor) {
    RelativeEncoder encoder = motor.getEncoder();
    encoder.setPosition(0);
    encoder.setPositionConversionFactor(conversionFactor);
    return encoder;
  }

  private void initPIDController(
      SparkMaxPIDController armPIDController,
      String armName,
      double p,
      double i,
      double d,
      double ff,
      double minOutput,
      double maxOutput,
      double maxVelocity,
      double maxAccel,
      double errorRange) {
    // Set PID coefficients
    armPIDController.setP(p);
    armPIDController.setI(i);
    armPIDController.setD(d);
    armPIDController.setIZone(0);
    armPIDController.setFF(ff);
    armPIDController.setOutputRange(minOutput, maxOutput);

    // Add Smart Motion configuration
    armPIDController.setSmartMotionMaxVelocity(maxVelocity, 0);
    armPIDController.setSmartMotionMaxAccel(maxAccel, 0);
    armPIDController.setSmartMotionAllowedClosedLoopError(errorRange, 0);

    // Display PID
    SmartDashboard.putNumber(armName + "P", p);
    SmartDashboard.putNumber(armName + "I", i);
    SmartDashboard.putNumber(armName + "D", d);
    SmartDashboard.putNumber(armName + "Max", maxOutput);
    SmartDashboard.putNumber(armName + "Min", minOutput);
    SmartDashboard.putNumber(armName + "FF", ff);
    SmartDashboard.putNumber(armName + "Rotation", 0);
  }

  private void setPeriodicFramePeriods(CANSparkMax motor) {
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);
  }
}