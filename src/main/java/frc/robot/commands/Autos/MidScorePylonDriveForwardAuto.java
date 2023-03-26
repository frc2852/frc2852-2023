// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Arm.DrivePositionCommand;
import frc.robot.commands.Arm.ScorePositionMidPylonCommand;
import frc.robot.commands.drive.DriveForwardDistanceCommand;
import frc.robot.commands.Intake.OpenIntakeCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class MidScorePylonDriveForwardAuto extends SequentialCommandGroup {
  public MidScorePylonDriveForwardAuto(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem,
      IntakeSubsystem intakeSubsystem) {

    addCommands(new DrivePositionCommand(armSubsystem));

    addCommands(new ScorePositionMidPylonCommand(armSubsystem));

    addCommands(new DriveForwardDistanceCommand(driveSubsystem, -Constants.ONE_FOOT, 0));
    addCommands(new OpenIntakeCommand(intakeSubsystem));

    addCommands(new DriveForwardDistanceCommand(driveSubsystem, Constants.ONE_FOOT, 0));
    addCommands(new DrivePositionCommand(armSubsystem));

    addCommands(new DriveForwardDistanceCommand(driveSubsystem, Constants.ONE_FOOT * 14, 0));
  }
}
