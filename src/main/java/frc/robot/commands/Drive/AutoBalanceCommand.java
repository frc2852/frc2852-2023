// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.AutoBalance;
import frc.robot.utils.AutoBalancePigeon;

public class AutoBalanceCommand extends CommandBase {

  private final DriveSubsystem mDriveSubsystem;

  private final AutoBalance mAutoBalance;
  // private final AutoBalancePigeon mAutoBalance;

  private final boolean mDriveReverse;

  public AutoBalanceCommand(DriveSubsystem driveSubsystem, Boolean driveReverse) {
    mDriveSubsystem = driveSubsystem;
    mDriveReverse = driveReverse;

    mAutoBalance = new AutoBalance();
    // mAutoBalance = new AutoBalancePigeon();

    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = mAutoBalance.autoBalanceRoutine(mDriveReverse);
    mDriveSubsystem.SetAutoSpeed(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
