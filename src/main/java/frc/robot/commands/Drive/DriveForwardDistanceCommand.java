// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DriveForwardDistanceCommand extends CommandBase {

	private final DriveSubsystem mDriveSubsystem;
  private final double mDistanceToTravel;
  private final double mSpeed;

	public DriveForwardDistanceCommand(DriveSubsystem driveSubsystem, double distanceToTravel, double speed) {
		mDriveSubsystem = driveSubsystem;
    mDistanceToTravel = distanceToTravel;
    mSpeed = speed;
		addRequirements(driveSubsystem);
	}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mDriveSubsystem.DriveForwardInches(mDistanceToTravel, mSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mDriveSubsystem.ResetAuto();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return mDriveSubsystem.IsAutoDriveFinished();
  }
}
