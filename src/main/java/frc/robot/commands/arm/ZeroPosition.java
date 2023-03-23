// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.pivots.InnerArmPosition;
import frc.robot.commands.arm.pivots.OuterArmPosition;
import frc.robot.commands.arm.pivots.WristPosition;
import frc.robot.subsystems.ArmSubsystem;

public class ZeroPosition extends SequentialCommandGroup {
  public ZeroPosition(ArmSubsystem armSubsystem) {
    addCommands(new WristPosition(armSubsystem, 0, 0));
    addCommands(new OuterArmPosition(armSubsystem, 0, 0));
    addCommands(new InnerArmPosition(armSubsystem, 0, 0.1));
  }
}
