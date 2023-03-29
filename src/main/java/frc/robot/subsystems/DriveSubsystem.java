// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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
  private RelativeEncoder mLeftEncoder;

  private RelativeEncoder mRightEncoder;

  private DifferentialDrive mDifferentialDrive = null;
  private final DoubleSolenoid mShifter;

  private boolean mIsHighGear;

  // Move to constants
  private final double WHEEL_DIAMETER_INCHES = 6.0;
  private final double GEAR_RATIO = 15.0;
  private final double ENCODER_CONVERSION_FACTOR = (1 / (WHEEL_DIAMETER_INCHES * Math.PI)) * GEAR_RATIO;

  // Auto
  private double mDistanceToTravelInches = 0;
  private double autoSpeed = 0;
  private boolean mAutoStarted = false;

  public DriveSubsystem() {
    mLeftLeader = new CANSparkMax(Constants.DRIVE_LEFT_LEADER, MotorType.kBrushless);
    mLeftLeader.setInverted(false);
    mLeftLeader.enableVoltageCompensation(12.0);
    mLeftLeader.setClosedLoopRampRate(Constants.DRIVE_VOLTAGE_RAMP_RATE);
    mLeftLeader.setIdleMode(IdleMode.kBrake);
    mLeftLeader.burnFlash();

    mLeftEncoder = mLeftLeader.getEncoder();
    mLeftEncoder.setPosition(0);
    mLeftEncoder.setPositionConversionFactor(ENCODER_CONVERSION_FACTOR);

    mLeftFollower = new CANSparkMax(Constants.DRIVE_LEFT_FOLLOWER, MotorType.kBrushless);
    mLeftFollower.setInverted(false);
    mLeftFollower.enableVoltageCompensation(12.0);
    mLeftFollower.setClosedLoopRampRate(Constants.DRIVE_VOLTAGE_RAMP_RATE);
    mLeftFollower.setIdleMode(IdleMode.kBrake);
    mLeftFollower.follow(mLeftLeader);
    mLeftFollower.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
    mLeftFollower.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
    mLeftFollower.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);
    mLeftFollower.burnFlash();

    mRightLeader = new CANSparkMax(Constants.DRIVE_RIGHT_LEADER, MotorType.kBrushless);
    mRightLeader.setInverted(false);
    mRightLeader.enableVoltageCompensation(12.0);
    mRightLeader.setClosedLoopRampRate(Constants.DRIVE_VOLTAGE_RAMP_RATE);
    mRightLeader.setIdleMode(IdleMode.kBrake);
    mRightLeader.burnFlash();

    mRightEncoder = mRightLeader.getEncoder();
    mRightEncoder.setPosition(0);
    mRightEncoder.setPositionConversionFactor(ENCODER_CONVERSION_FACTOR);

    mRightFollower = new CANSparkMax(Constants.DRIVE_RIGHT_FOLLOWER, MotorType.kBrushless);
    mRightFollower.setInverted(false);
    mRightFollower.enableVoltageCompensation(12.0);
    mRightFollower.setClosedLoopRampRate(Constants.DRIVE_VOLTAGE_RAMP_RATE);
    mRightFollower.setIdleMode(IdleMode.kBrake);
    mRightFollower.follow(mRightLeader);
    mRightFollower.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
    mRightFollower.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
    mRightFollower.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);
    mRightFollower.burnFlash();

    mShifter = new DoubleSolenoid(Constants.PNEUMATIC_HUB, PneumaticsModuleType.REVPH, Constants.DRIVE_GEAR_BOX_OPEN,
        Constants.DRIVE_GEAR_BOX_CLOSE);

    SetLowGear();

    mDifferentialDrive = new DifferentialDrive(mLeftLeader, mRightLeader);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("High Gear", mIsHighGear);

    if (DriverStation.isAutonomous()) {
      // Dumb auto drive

      if (mDistanceToTravelInches != 0) {

        double distancePosition = mDistanceToTravelInches * 0.27; // 3.24
        double distance = distancePosition - mLeftEncoder.getPosition();
        SmartDashboard.putNumber("Distance", distance);

        if (distance > 0.5) {
          double left = autoSpeed == 0 ? 0.39 : autoSpeed;
          double right = autoSpeed == 0 ? -0.4 : -autoSpeed;

          mDifferentialDrive.tankDrive(left, right);
        } else if (distance < -0.5) {
          double left = autoSpeed == 0 ? -0.39 : -autoSpeed;
          double right = autoSpeed == 0 ? 0.4 : autoSpeed;

          mDifferentialDrive.tankDrive(left, right);
        } else {
          mDifferentialDrive.tankDrive(0, 0);
          mDistanceToTravelInches = 0;
        }
      } else {
        mDifferentialDrive.tankDrive(0, 0);
      }

    }
  }

  public void DriveForwardInches(double distanceToTravelInches, double speed) {
    mDistanceToTravelInches = distanceToTravelInches;
    autoSpeed = speed;
    mAutoStarted = true;
  }

  public boolean IsAutoDriveFinished() {
    return (mAutoStarted && mDistanceToTravelInches == 0);
  }

  public void ResetAuto() {
    mAutoStarted = false;
    mDistanceToTravelInches = 0;
  }

  public void ArcadeDrive(double xSpeed, double zRotation) {
    if (DriverStation.isTeleop()) {
      mDifferentialDrive.arcadeDrive(xSpeed * 1.0, zRotation * 1.0);
    }
  }

  public void ToggleGear() {
    if (mIsHighGear) {
      SetLowGear();
    } else {
      SetHighGear();
    }
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
}
