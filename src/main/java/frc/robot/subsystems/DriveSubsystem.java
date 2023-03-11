// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.libraries.SparkMaxExtended;

public class DriveSubsystem extends SubsystemBase {

  // Hardware
  private final SparkMaxExtended mLeftLeader, mRightLeader, mLeftFollower, mRightFollower;
  private DifferentialDrive mDifferentialDrive;
  private final DoubleSolenoid mShifter;

  private boolean mIsHighGear;

  private void configureSpark(SparkMaxExtended sparkMax, boolean inverted) {
    sparkMax.setInverted(inverted);
    sparkMax.enableVoltageCompensation(12.0);
    sparkMax.setClosedLoopRampRate(Constants.DRIVE_VOLTAGE_RAMP_RATE);
    sparkMax.setIdleMode(IdleMode.kBrake);
    sparkMax.burnFlash();
  }

  public DriveSubsystem(CommandXboxController driveController) {
    
    mLeftLeader = new SparkMaxExtended(Constants.DRIVE_LEFT_LEADER);
    configureSpark(mLeftLeader, false);

    mLeftFollower = new SparkMaxExtended(Constants.DRIVE_LEFT_FOLLOWER, mLeftLeader);
    configureSpark(mLeftFollower, false);

    mRightLeader = new SparkMaxExtended(Constants.DRIVE_RIGHT_LEADER);
    configureSpark(mRightLeader, false);

    mRightFollower = new SparkMaxExtended(Constants.DRIVE_RIGHT_FOLLOWER, mRightLeader);
    configureSpark(mRightFollower, false);

    mShifter = new DoubleSolenoid(Constants.PNEUMATIC_HUB, PneumaticsModuleType.REVPH, Constants.DRIVE_GEAR_BOX_OPEN, Constants.DRIVE_GEAR_BOX_CLOSE);

    SetLowGear();

    mDifferentialDrive = new DifferentialDrive(mLeftLeader, mRightLeader);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("High Gear", mIsHighGear);
  }

  public void ArcadeDrive(double xSpeed, double zRotation) {
		SmartDashboard.putNumber("xSpeed: ", xSpeed);
		SmartDashboard.putNumber("zRotation: ", zRotation);
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
    mShifter.set(DoubleSolenoid.Value.kReverse);
    mIsHighGear = false;
  }

  public void SetHighGear() {
    mShifter.set(DoubleSolenoid.Value.kForward);
    mIsHighGear = true;
  }
}
