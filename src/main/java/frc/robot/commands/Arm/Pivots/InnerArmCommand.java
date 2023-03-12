// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm.Pivots;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class InnerArmCommand extends CommandBase {

  private final ArmSubsystem mArmSubsystem;
  private final double mTargetPosition;
  private final double mSpeed;

  /** Creates a new OuterArmCommand. */
  public InnerArmCommand(ArmSubsystem armSubsystem, double targetPosition, double speed) {
    mArmSubsystem = armSubsystem;
    mTargetPosition = targetPosition;
    mSpeed = speed;
    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mArmSubsystem.SetInnerArmPosition(mTargetPosition, mSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return mArmSubsystem.IsInnerArmAtPosition();
  }
}
