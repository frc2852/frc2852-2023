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

public class HighPickupPositionCommand extends SequentialCommandGroup {
  public HighPickupPositionCommand(ArmSubsystem armSubsystem) {

    if (ArmSubsystem.armPosition != ArmPosition.DRIVE && 
        ArmSubsystem.armPosition != ArmPosition.PICK_UP &&
        ArmSubsystem.armPosition != ArmPosition.HIGH_PICK_UP) {
      addCommands(new DrivePositionCommand(armSubsystem));
    }

    ArmSubsystem.armPosition = ArmPosition.HIGH_PICK_UP;
    addCommands(new InnerArmCommand(armSubsystem, 14.9, 0.4));
    addCommands(new WristCommand(armSubsystem, -21.3, 0)); //Test if this can run last, after outerArm
    addCommands(new OuterArmCommand(armSubsystem, -15.5, 0));
  }
}
