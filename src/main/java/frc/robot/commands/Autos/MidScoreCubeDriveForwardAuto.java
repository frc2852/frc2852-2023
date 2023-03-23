// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.arm.DrivePosition;
import frc.robot.commands.arm.ScorePositionMidCube;
import frc.robot.commands.drive.DriveForwardDistance;
import frc.robot.commands.intake.TimedOuttake;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class MidScoreCubeDriveForwardAuto extends SequentialCommandGroup {
  public MidScoreCubeDriveForwardAuto(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem,
      IntakeSubsystem intakeSubsystem) {
    addCommands(new DrivePosition(armSubsystem));

    addCommands(new ScorePositionMidCube(armSubsystem));

    addCommands(new DriveForwardDistance(driveSubsystem, -10));
    addCommands(new TimedOuttake(intakeSubsystem, 2));

    addCommands(new DriveForwardDistance(driveSubsystem, Constants.ONE_FOOT));
    addCommands(new DrivePosition(armSubsystem));

    addCommands(new DriveForwardDistance(driveSubsystem, Constants.ONE_FOOT * 14));
  }
}
