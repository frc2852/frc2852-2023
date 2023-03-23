// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmPosition;
import frc.robot.commands.arm.pivots.InnerArmPosition;
import frc.robot.commands.arm.pivots.OuterArmPosition;
import frc.robot.commands.arm.pivots.WristPosition;
import frc.robot.subsystems.ArmSubsystem;

public class ScorePositionLow extends SequentialCommandGroup {
  /** Creates a new LowScorePositionCommand. */
  public ScorePositionLow(ArmSubsystem armSubsystem) {

    addCommands(new InstantCommand(() -> {
      if(ArmSubsystem.ARM_POSITION == ArmPosition.PICK_UP || ArmSubsystem.ARM_POSITION == ArmPosition.HIGH_PICK_UP){
        new DrivePosition(armSubsystem).schedule();
      }
    }));

    ArmSubsystem.ARM_POSITION = ArmPosition.LOW_GOAL;
    addCommands(new InnerArmPosition(armSubsystem, -7, 0.4));
    addCommands(new WristPosition(armSubsystem, -13, 0));
    addCommands(new OuterArmPosition(armSubsystem, -7.5, 0));

  }
}
