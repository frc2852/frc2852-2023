// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.pivots.InnerArmCommand;
import frc.robot.commands.arm.pivots.OuterArmCommand;
import frc.robot.commands.arm.pivots.WristCommand;
import frc.robot.subsystems.ArmSubsystem;

public class ZeroPositionCommand extends SequentialCommandGroup {
  public ZeroPositionCommand(ArmSubsystem armSubsystem) {
    addCommands(new WristCommand(armSubsystem, 0, 0));
    addCommands(new OuterArmCommand(armSubsystem, 0, 0));
    addCommands(new InnerArmCommand(armSubsystem, 0, 0.1));
  }
}
