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

public class DrivePosition extends SequentialCommandGroup {
  /** Creates a new DrivePositionCommand. */
  public DrivePosition(ArmSubsystem armSubsystem) {

    addCommands(new InstantCommand(() -> {
      if (ArmSubsystem.ARM_POSITION == ArmPosition.PICK_UP) {
        new InnerArmPosition(armSubsystem, -7.5, 0.1).schedule();
      }
    }));

    ArmSubsystem.ARM_POSITION = ArmPosition.DRIVE;
    addCommands(new WristPosition(armSubsystem, 3, 0));
    addCommands(new OuterArmPosition(armSubsystem, -5, 0));
    addCommands(new InnerArmPosition(armSubsystem, -4.5, 0.1));
  }
}
