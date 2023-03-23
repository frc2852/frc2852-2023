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

public class ScorePositionHighCube extends SequentialCommandGroup {
  /** Creates a new LowScorePositionCommand. */
  public ScorePositionHighCube(ArmSubsystem armSubsystem) {

    addCommands(new InstantCommand(() -> {
      if (ArmSubsystem.ARM_POSITION == ArmPosition.PICK_UP || ArmSubsystem.ARM_POSITION == ArmPosition.HIGH_PICK_UP) {
        new DrivePosition(armSubsystem).schedule();
      }
    }));

    addCommands(new InstantCommand(() -> {
      if (ArmSubsystem.ARM_POSITION != ArmPosition.DRIVE && ArmSubsystem.ARM_POSITION != ArmPosition.MID_GOAL
          && ArmSubsystem.ARM_POSITION != ArmPosition.HIGH_GOAL) {
        new WristPosition(armSubsystem, 2, 0).schedule();
      }
    }));

    ArmSubsystem.ARM_POSITION = ArmPosition.HIGH_GOAL;
    addCommands(new InnerArmPosition(armSubsystem, -19, 0.4));
    addCommands(new WristPosition(armSubsystem, -20, 0));
    addCommands(new OuterArmPosition(armSubsystem, -3.5, 0));

  }
}
