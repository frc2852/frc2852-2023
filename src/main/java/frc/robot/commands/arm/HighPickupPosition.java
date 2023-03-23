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

public class HighPickupPosition extends SequentialCommandGroup {
  public HighPickupPosition(ArmSubsystem armSubsystem) {

    addCommands(new InstantCommand(() -> {
      if (ArmSubsystem.ARM_POSITION != ArmPosition.DRIVE &&
          ArmSubsystem.ARM_POSITION != ArmPosition.PICK_UP &&
          ArmSubsystem.ARM_POSITION != ArmPosition.HIGH_PICK_UP) {
        new DrivePosition(armSubsystem).schedule();
      }
    }));

    ArmSubsystem.ARM_POSITION = ArmPosition.HIGH_PICK_UP;
    addCommands(new InnerArmPosition(armSubsystem, 14.9, 0.4));
    addCommands(new WristPosition(armSubsystem, -21.3, 0)); // Test if this can run last, after outerArm
    addCommands(new OuterArmPosition(armSubsystem, -15.5, 0));
  }
}
