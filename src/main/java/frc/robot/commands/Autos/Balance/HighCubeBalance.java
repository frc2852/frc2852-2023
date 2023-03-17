// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos.Balance;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Arm.DrivePositionCommand;
import frc.robot.commands.Arm.ScorePositionHighCubeCommand;
import frc.robot.commands.Drive.AutoBalanceCommand;
import frc.robot.commands.Drive.DriveForwardDistanceCommand;
import frc.robot.commands.Intake.TimedOuttakeCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class HighCubeBalance extends SequentialCommandGroup {
  public HighCubeBalance(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem) {
    addCommands(new DrivePositionCommand(armSubsystem));

    addCommands(new ScorePositionHighCubeCommand(armSubsystem));

    addCommands(new DriveForwardDistanceCommand(driveSubsystem, -Constants.ONE_FOOT));
    addCommands(new TimedOuttakeCommand(intakeSubsystem, 1));

    double distanceToRamp = 40; //TODO
    addCommands(new DriveDistanceCommand(driveSubsystem, distanceToRamp, 0.78, 0.8));
    addCommands(new AutoBalanceCommand(driveSubsystem, false));
  }
}
