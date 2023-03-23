// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {

  // Hardware
  private final CANSparkMax mLeftLeader, mRightLeader, mLeftFollower, mRightFollower;
  private final RelativeEncoder mLeftEncoder, mRightEncoder;
  private final DifferentialDrive mDifferentialDrive;
  private final DoubleSolenoid mShifter;
  private final PigeonIMU mPigeonIMU;

  private boolean mIsHighGear;
  private double mDistanceToTravelInches = 0;
  private boolean mAutoStarted = false;

  public DriveSubsystem() {
    mLeftLeader = initMotor(Constants.DRIVE_LEFT_LEADER, false, null);
    mLeftFollower = initMotor(Constants.DRIVE_LEFT_FOLLOWER, false, mLeftLeader);
    mLeftEncoder = initEncoder(mLeftLeader);

    mRightLeader = initMotor(Constants.DRIVE_RIGHT_LEADER, true, null);
    mRightFollower = initMotor(Constants.DRIVE_RIGHT_FOLLOWER, true, mRightLeader);
    mRightEncoder = initEncoder(mRightLeader);

    mShifter = new DoubleSolenoid(Constants.PNEUMATIC_HUB, PneumaticsModuleType.REVPH, Constants.DRIVE_GEAR_BOX_OPEN,
        Constants.DRIVE_GEAR_BOX_CLOSE);

    mPigeonIMU = new PigeonIMU(Constants.PIGEON_IMU);

    SetLowGear();
    mDifferentialDrive = new DifferentialDrive(mLeftLeader, mRightLeader);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("High Gear", mIsHighGear);

    if (DriverStation.isAutonomous()) {
      // Encoder-based auto drive
      if (mDistanceToTravelInches != 0) {
        double leftDistance = mDistanceToTravelInches - mLeftEncoder.getPosition();
        double rightDistance = mDistanceToTravelInches - mRightEncoder.getPosition();

        SmartDashboard.putNumber("Left Distance", leftDistance);
        SmartDashboard.putNumber("Right Distance", rightDistance);

        double baseSpeed = 0.5;
        double correctionFactor = 0.05; // Adjust this value to change the sensitivity of the correction

        if (Math.abs(leftDistance) > 0.5 || Math.abs(rightDistance) > 0.5) {
          double error = leftDistance - rightDistance;
          double correction = error * correctionFactor;
          double leftSpeed = baseSpeed + correction;
          double rightSpeed = baseSpeed - correction;

          mDifferentialDrive.tankDrive(leftSpeed, rightSpeed);
        } else {
          mDifferentialDrive.tankDrive(0, 0);
          mDistanceToTravelInches = 0;
        }
      } else {
        mDifferentialDrive.tankDrive(0, 0);
      }
    }
  }

  public void DriveForwardInches(double distanceToTravelInches) {
    mDistanceToTravelInches = distanceToTravelInches;
    mAutoStarted = true;
  }

  public boolean IsAutoDriveFinished() {
    return (mAutoStarted && mDistanceToTravelInches == 0);
  }

  public void ResetAuto() {
    mAutoStarted = false;
    mDistanceToTravelInches = 0;
  }

  public void ArcadeDrive(double xSpeed, double zRotation, boolean allowTurnInPlace) {
    if (DriverStation.isTeleop()) {
      // mDifferentialDrive.arcadeDrive(xSpeed, zRotation);
      mDifferentialDrive.curvatureDrive(xSpeed, zRotation, allowTurnInPlace);
    }
  }

  public void ToggleGear() {
    if (mIsHighGear) {
      SetLowGear();
    } else {
      SetHighGear();
    }
  }

  public void AutoBalanceDrive(boolean halfSpeed) {
    // Get the current pitch from the Pigeon IMU
    double[] ypr_deg = new double[3];
    mPigeonIMU.getYawPitchRoll(ypr_deg);
    double curPitch = Math.sin(Math.toRadians(ypr_deg[1]));

    // Proportional gain
    double pGain = 2.0;

    // Calculate the output using the proportional gain
    double output = -curPitch * pGain;

    // Set the maximum output based on the halfSpeed flag
    double maxOut = halfSpeed ? 0.5 : 1.0;

    // Apply the maximum output limits
    output = Math.max(-maxOut, Math.min(output, maxOut));

    // Set the output for both left and right drives using tankDrive
    mDifferentialDrive.tankDrive(output, output);
  }

  public void SetLowGear() {
    mLeftLeader.setIdleMode(IdleMode.kBrake);
    mLeftFollower.setIdleMode(IdleMode.kBrake);
    mRightLeader.setIdleMode(IdleMode.kBrake);
    mRightFollower.setIdleMode(IdleMode.kBrake);
    mShifter.set(DoubleSolenoid.Value.kReverse);
    mIsHighGear = false;
  }

  public void SetHighGear() {
    mLeftLeader.setIdleMode(IdleMode.kCoast);
    mLeftFollower.setIdleMode(IdleMode.kCoast);
    mRightLeader.setIdleMode(IdleMode.kCoast);
    mRightFollower.setIdleMode(IdleMode.kCoast);
    mShifter.set(DoubleSolenoid.Value.kForward);
    mIsHighGear = true;
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

  private RelativeEncoder initEncoder(CANSparkMax motor) {
    RelativeEncoder encoder = motor.getEncoder();
    encoder.setPosition(0);
    encoder.setPositionConversionFactor(Constants.DRIVE_ENCODER_CONVERSION_FACTOR);
    return encoder;
  }

  private void setPeriodicFramePeriods(CANSparkMax motor) {
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);
  }
}
