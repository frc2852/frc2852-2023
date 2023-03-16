// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {

  // Hardware
  private final CANSparkMax mLeftLeader, mRightLeader, mLeftFollower, mRightFollower;
  private RelativeEncoder mLeftEncoder;
  private SparkMaxPIDController mLeftPIDController;

  private RelativeEncoder mRightEncoder;
  private SparkMaxPIDController mRightPIDController;

  private DifferentialDrive mDifferentialDrive = null;
  private final DoubleSolenoid mShifter;

  private boolean mIsHighGear;

  private final double WHEEL_DIAMETER_INCHES = 6.0;
  private final double GEAR_RATIO = 15.0;
  private final double ENCODER_CONVERSION_FACTOR = (1 / (WHEEL_DIAMETER_INCHES * Math.PI)) * GEAR_RATIO;
  private double mDistanceToTravelInches = 0;

  private void configureSpark(CANSparkMax sparkMax, boolean inverted) {
    sparkMax.setInverted(inverted);
    sparkMax.enableVoltageCompensation(12.0);
    sparkMax.setClosedLoopRampRate(Constants.DRIVE_VOLTAGE_RAMP_RATE);
    sparkMax.setIdleMode(IdleMode.kBrake);
    sparkMax.burnFlash();
  }

  public DriveSubsystem() {

    mLeftLeader = new CANSparkMax(Constants.DRIVE_LEFT_LEADER, MotorType.kBrushless);
    configureSpark(mLeftLeader, false);

    mLeftPIDController = mLeftLeader.getPIDController();
    mLeftPIDController.setP(0.1);
    mLeftPIDController.setI(0);
    mLeftPIDController.setD(0);
    mLeftPIDController.setIZone(0);
    mLeftPIDController.setFF(0);
    mLeftPIDController.setOutputRange(-0.2, 0.2);

    mLeftEncoder = mLeftLeader.getEncoder();
    mLeftEncoder.setPosition(0);
    mLeftEncoder.setPositionConversionFactor(ENCODER_CONVERSION_FACTOR);

    mLeftFollower = new CANSparkMax(Constants.DRIVE_LEFT_FOLLOWER, MotorType.kBrushless);
    configureSpark(mLeftFollower, false);
    mLeftFollower.follow(mLeftLeader);

    mRightLeader = new CANSparkMax(Constants.DRIVE_RIGHT_LEADER, MotorType.kBrushless);
    configureSpark(mRightLeader, false);

    mRightPIDController = mRightLeader.getPIDController();
    mRightPIDController.setP(0.1);
    mRightPIDController.setI(0);
    mRightPIDController.setD(0);
    mRightPIDController.setIZone(0);
    mRightPIDController.setFF(0);
    mRightPIDController.setOutputRange(-0.2, 0.2);

    mRightEncoder = mRightLeader.getEncoder();
    mRightEncoder.setPosition(0);
    mRightEncoder.setPositionConversionFactor(ENCODER_CONVERSION_FACTOR);

    mRightFollower = new CANSparkMax(Constants.DRIVE_RIGHT_FOLLOWER, MotorType.kBrushless);
    configureSpark(mRightFollower, false);
    mRightFollower.follow(mRightLeader);

    mShifter = new DoubleSolenoid(Constants.PNEUMATIC_HUB, PneumaticsModuleType.REVPH, Constants.DRIVE_GEAR_BOX_OPEN,
        Constants.DRIVE_GEAR_BOX_CLOSE);

    SetLowGear();

    mDifferentialDrive = new DifferentialDrive(mLeftLeader, mRightLeader);

    mDistanceToTravelInches = 12 * 15;
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("High Gear", mIsHighGear);

    //Bad auto
    if (mDistanceToTravelInches > 0) {
      double distancePosition = mDistanceToTravelInches * 0.27; // 3.24
      double distance = distancePosition - mLeftEncoder.getPosition();
      if (distance > 0.5) {
        mDifferentialDrive.tankDrive(0.39, -0.4);
      } else {
        mDifferentialDrive.tankDrive(0, 0);
        mDistanceToTravelInches = 0;
      }
    }
  }

  public void DriveForwardInches(double distanceToTravelInches) {
    mDistanceToTravelInches = distanceToTravelInches;
  }

  public void ArcadeDrive(double xSpeed, double zRotation) {
    if (mDistanceToTravelInches == 0) {
      mDifferentialDrive.arcadeDrive(xSpeed, zRotation);
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
