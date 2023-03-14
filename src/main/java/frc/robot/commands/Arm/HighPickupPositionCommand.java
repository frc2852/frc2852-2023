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

    ArmSubsystem.armPosition = ArmPosition.HIGH_PICK_UP;
    addCommands(new InnerArmCommand(armSubsystem, 0.0, 0.1));
    addCommands(new OuterArmCommand(armSubsystem, 0.0, 0));
    addCommands(new WristCommand(armSubsystem, 0.0, 0));
  }
}

