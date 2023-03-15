// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Arm.ScorePositionHighCubeCommand;
import frc.robot.commands.Intake.TimedOuttakeCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class HighScoreCubeDriveForwardAuto extends SequentialCommandGroup {
  public HighScoreCubeDriveForwardAuto(ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem) {
    addCommands(new ScorePositionHighCubeCommand(armSubsystem));
    addCommands(new TimedOuttakeCommand(intakeSubsystem, 2));
  }
}
