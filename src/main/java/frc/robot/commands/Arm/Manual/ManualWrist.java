// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm.manual;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ManualWrist extends CommandBase {
  /** Creates a new ManualInner. */

  private boolean mForward;

  public ManualWrist(boolean forward) {
    mForward = forward;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (mForward) {
      ArmSubsystem.WRIST_POSITION = ArmSubsystem.WRIST_POSITION + 1;
    } else {
      ArmSubsystem.WRIST_POSITION = ArmSubsystem.WRIST_POSITION - 1;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
