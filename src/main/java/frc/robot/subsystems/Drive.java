// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.libraries.LazySparkMax;
import frc.robot.libraries.SparkMaxFactory;

public class Drive extends SubsystemBase {

  // Hardware
  private final LazySparkMax mLeftMaster, mRightMaster, mLeftSlave, mRightSlave;
  private DifferentialDrive mDifferentialDrive;
  
  private final Solenoid mShifter;

  private boolean mIsHighGear;
  private boolean mIsBrakeMode;

  private void configureSpark(LazySparkMax sparkMax, boolean left, boolean master) {
    sparkMax.setInverted(!left);
    sparkMax.enableVoltageCompensation(12.0);
    sparkMax.setClosedLoopRampRate(Constants.DRIVE_VOLTAGE_RAMP_RATE);
  }

  public Drive() {
    // start all Talons in open loop mode
    mLeftMaster = SparkMaxFactory.createDefaultSparkMax(Constants.DRIVE_LEFT_MASTER);
    configureSpark(mLeftMaster, true, true);

    mLeftSlave = SparkMaxFactory.createPermanentSlaveSparkMax(Constants.DRIVE_LEFT_SLAVE, mLeftMaster);
    configureSpark(mLeftSlave, true, false);

    mRightMaster = SparkMaxFactory.createDefaultSparkMax(Constants.DRIVE_RIGHT_MASTER);
    configureSpark(mRightMaster, false, true);

    mRightSlave = SparkMaxFactory.createPermanentSlaveSparkMax(Constants.DRIVE_RIGHT_SLAVE, mRightMaster);
    configureSpark(mRightSlave, false, false);

    mShifter = new Solenoid(PneumaticsModuleType.REVPH, Constants.DRIVE_SHIFT);

    // force a solenoid message
    mIsHighGear = false;
    setHighGear(true);

    // force a CAN message across
    mIsBrakeMode = true;
    setBrakeMode(false);

    mDifferentialDrive = new DifferentialDrive(mLeftMaster, mRightMaster);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void ArcadeDrive(double xSpeed, double zRotation) {
		SmartDashboard.putNumber("xSpeed: ", xSpeed);
		SmartDashboard.putNumber("zRotation: ", zRotation);
		mDifferentialDrive.arcadeDrive(xSpeed, zRotation);
	}

  public synchronized void setHighGear(boolean wantsHighGear) {
    if (wantsHighGear != mIsHighGear) {
      mIsHighGear = wantsHighGear;
      // Plumbed default high.
      mShifter.set(!wantsHighGear);
    }
  }

  public synchronized void setBrakeMode(boolean shouldEnable) {
    if (mIsBrakeMode != shouldEnable) {
      mIsBrakeMode = shouldEnable;
      IdleMode mode = shouldEnable ? IdleMode.kBrake : IdleMode.kCoast;
      mRightMaster.setIdleMode(mode);
      mRightSlave.setIdleMode(mode);

      mLeftMaster.setIdleMode(mode);
      mLeftSlave.setIdleMode(mode);
    }
  }
}
