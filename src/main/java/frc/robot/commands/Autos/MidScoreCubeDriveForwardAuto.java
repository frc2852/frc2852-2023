// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.arm.DrivePositionCommand;
import frc.robot.commands.arm.ScorePositionMidCubeCommand;
import frc.robot.commands.drive.DriveForwardDistanceCommand;
import frc.robot.commands.intake.TimedOuttakeCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class MidScoreCubeDriveForwardAuto extends SequentialCommandGroup {
  public MidScoreCubeDriveForwardAuto(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem,
      IntakeSubsystem intakeSubsystem) {
    addCommands(new DrivePositionCommand(armSubsystem));

    addCommands(new ScorePositionMidCubeCommand(armSubsystem));

    addCommands(new DriveForwardDistanceCommand(driveSubsystem, -10));
    addCommands(new TimedOuttakeCommand(intakeSubsystem, 2));

    addCommands(new DriveForwardDistanceCommand(driveSubsystem, Constants.ONE_FOOT));
    addCommands(new DrivePositionCommand(armSubsystem));

    addCommands(new DriveForwardDistanceCommand(driveSubsystem, Constants.ONE_FOOT * 14));
  }
}
