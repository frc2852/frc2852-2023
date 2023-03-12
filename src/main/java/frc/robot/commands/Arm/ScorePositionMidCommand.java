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

public class ScorePositionMidCommand extends SequentialCommandGroup {
  /** Creates a new LowScorePositionCommand. */
  public ScorePositionMidCommand(ArmSubsystem armSubsystem) {
    if(ArmSubsystem.armPosition == ArmPosition.PICK_UP){
      addCommands(new DrivePositionCommand(armSubsystem));
    }
    
    ArmSubsystem.armPosition = ArmPosition.MID_GOAL;
    addCommands(new WristCommand(armSubsystem, 12, 0));
    addCommands(new OuterArmCommand(armSubsystem, -5, 0));
    addCommands(new InnerArmCommand(armSubsystem, -19, 0.3));
  }
}
