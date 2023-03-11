// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Drive.DriveCommand;
import frc.robot.commands.Drive.ToggleGear;
import frc.robot.commands.Intake.IntakeCubeCommand;
import frc.robot.commands.Intake.IntakePylonCommand;
import frc.robot.commands.Intake.OuttakeCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems and commands are defined here...
  private final CommandXboxController driverController = new CommandXboxController(Constants.DRIVER_CONTROLLER);
  private final CommandXboxController operatorController = new CommandXboxController(Constants.OPERATOR_CONTROLLER);

  private final DriveSubsystem mDriveSubsystem = new DriveSubsystem();
  private final IntakeSubsystem mIntakeSubsystem = new IntakeSubsystem();
  private final ArmSubsystem mArmSubsystem = new ArmSubsystem();

  private final ToggleGear mToggleGearCommand = new ToggleGear(mDriveSubsystem);

  private final IntakeCubeCommand mIntakeCubeCommand = new IntakeCubeCommand(mIntakeSubsystem);
  private final IntakePylonCommand mIntakePylonCommand = new IntakePylonCommand(mIntakeSubsystem);
  private final OuttakeCommand mOuttakeCommand = new OuttakeCommand(mIntakeSubsystem);

  private final PneumaticHub mPneumaticHub = new PneumaticHub(Constants.PNEUMATIC_HUB);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    ConfigureDriveController();
    ConfigureOperatorController();
    mPneumaticHub.enableCompressorDigital();
  }

  private void ConfigureDriveController() {
    mDriveSubsystem.setDefaultCommand(
        new DriveCommand(mDriveSubsystem, () -> driverController.getLeftX(), () -> driverController.getLeftY()));

    driverController.b().toggleOnTrue(mToggleGearCommand);

    driverController.rightBumper().whileTrue(mIntakeCubeCommand);
    driverController.leftBumper().whileTrue(mIntakePylonCommand);
    driverController.rightTrigger().whileTrue(mOuttakeCommand);
  }

  private void ConfigureOperatorController() {
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
