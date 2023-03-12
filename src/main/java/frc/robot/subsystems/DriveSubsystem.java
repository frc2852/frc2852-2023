// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
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
  private DifferentialDrive mDifferentialDrive;
  private final DoubleSolenoid mShifter;

  private boolean mIsHighGear;

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

    mLeftFollower = new CANSparkMax(Constants.DRIVE_LEFT_FOLLOWER, MotorType.kBrushless);
    configureSpark(mLeftFollower, false);
    mLeftFollower.follow( mLeftLeader);

    mRightLeader = new CANSparkMax(Constants.DRIVE_RIGHT_LEADER, MotorType.kBrushless);
    configureSpark(mRightLeader, false);

    mRightFollower = new CANSparkMax(Constants.DRIVE_RIGHT_FOLLOWER, MotorType.kBrushless);
    configureSpark(mRightFollower, false);
    mRightFollower.follow(mLeftLeader);
    
    mShifter = new DoubleSolenoid(Constants.PNEUMATIC_HUB, PneumaticsModuleType.REVPH, Constants.DRIVE_GEAR_BOX_OPEN, Constants.DRIVE_GEAR_BOX_CLOSE);

    SetLowGear();

    mDifferentialDrive = new DifferentialDrive(mLeftLeader, mRightLeader);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("High Gear", mIsHighGear);
  }

  public void ArcadeDrive(double xSpeed, double zRotation) {
		mDifferentialDrive.arcadeDrive(xSpeed, zRotation);
	}

  public void ToggleGear() {
    if(mIsHighGear){
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
