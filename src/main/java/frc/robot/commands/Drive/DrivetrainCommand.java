// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;

public class DrivetrainCommand extends CommandBase {

	/* Creates a new DrivetrainCommand. */
	private final Drive mDriveSubsystem;
	private final DoubleSupplier mXSpeed;
	private final DoubleSupplier mZRotation;

	public DrivetrainCommand(Drive driveSubsystem, DoubleSupplier xSpeed, DoubleSupplier zRotation) {
		mDriveSubsystem = driveSubsystem;
		mXSpeed = xSpeed;
		mZRotation = zRotation;
		addRequirements(driveSubsystem);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		mDriveSubsystem.ArcadeDrive(mXSpeed.getAsDouble(), mZRotation.getAsDouble());
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
