// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmPosition;
import frc.robot.commands.Arm.Pivots.InnerArmCommand;
import frc.robot.commands.Arm.Pivots.OuterArmCommand;
import frc.robot.commands.Arm.Pivots.WristCommand;
import frc.robot.subsystems.ArmSubsystem;

public class ScorePositionHighCubeCommand extends SequentialCommandGroup {
  /** Creates a new LowScorePositionCommand. */
  public ScorePositionHighCubeCommand(ArmSubsystem armSubsystem) {
    if(ArmSubsystem.armPosition == ArmPosition.PICK_UP || ArmSubsystem.armPosition == ArmPosition.HIGH_PICK_UP){
      addCommands(new DrivePositionCommand(armSubsystem));
    }

    if (ArmSubsystem.armPosition != ArmPosition.DRIVE && ArmSubsystem.armPosition != ArmPosition.MID_GOAL && ArmSubsystem.armPosition != ArmPosition.HIGH_GOAL) {
      addCommands(new WristCommand(armSubsystem, 2, 0));
    }
    
    ArmSubsystem.armPosition = ArmPosition.HIGH_GOAL;
    addCommands(new InnerArmCommand(armSubsystem, -19, 0.4));
    addCommands(new WristCommand(armSubsystem, -20, 0));
    addCommands(new OuterArmCommand(armSubsystem, -3.5, 0));
    
  }
}
