// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Drive.DrivetrainCommand;
import frc.robot.commands.Drive.ToggleGear;
import frc.robot.subsystems.Drive;
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

  private final Drive mDriveSubsystem = new Drive(driverController);
  private final ToggleGear mToggleGearCommand = new ToggleGear(mDriveSubsystem);

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
        new DrivetrainCommand(mDriveSubsystem, () -> driverController.getLeftX(), () -> driverController.getLeftY()));

    driverController.b().toggleOnTrue(mToggleGearCommand);

    // driverController.rightBumper().toggleOnTrue(m_intake.ingestIntake(false));
    // driverController.rightBumper().toggleOnFalse(m_intake.stopIntake());

    // driverController.leftBumper().toggleOnTrue(m_intake.ingestIntake(true));
    // driverController.leftBumper().toggleOnFalse(m_intake.stopIntake());

    // driverController.rightTrigger().onTrue(m_intake.regurgitateIntake());
    // driverController.rightTrigger().onFalse(m_intake.stopIntake());

    // driverController.b().toggleOnTrue(m_drive.shiftGear());
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
