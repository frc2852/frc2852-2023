// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class TimedOuttakeCommand extends CommandBase {

  private final IntakeSubsystem mIntakeSubsystem;
  private final double mRunTime;
  private final Timer mTimer = new Timer();

  public TimedOuttakeCommand(IntakeSubsystem intakeSubsystem, double runTime) {
    mIntakeSubsystem = intakeSubsystem;
    mRunTime = runTime;
    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mTimer.reset();
    mTimer.start();
    mIntakeSubsystem.regurgitateIntake();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mIntakeSubsystem.stopIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return mTimer.hasElapsed(mRunTime);
  }
}
