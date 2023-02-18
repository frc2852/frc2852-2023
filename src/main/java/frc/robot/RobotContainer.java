// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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

  private final Drive m_drive = new Drive(driverController);

  private final PneumaticHub m_pneumaticHub = new PneumaticHub(Constants.PNEUMATIC_HUB);
  private final Intake m_intake = new Intake(driverController);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    m_pneumaticHub.enableCompressorDigital();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is
    // pressed,
    // cancelling on release.

    driverController
        .axisGreaterThan(Constants.LEFT_STICK_VERT, Constants.DEAD_ZONE)
        .whileTrue(
            m_drive.drive());

    driverController
        .axisLessThan(Constants.LEFT_STICK_VERT, -Constants.DEAD_ZONE)
        .whileTrue(
            m_drive.drive());

    driverController
        .axisGreaterThan(Constants.LEFT_STICK_HORZ, Constants.DEAD_ZONE)
        .whileTrue(
            m_drive.drive());

    driverController
        .axisLessThan(Constants.LEFT_STICK_HORZ, -Constants.DEAD_ZONE)
        .whileTrue(
            m_drive.drive());

    driverController.rightBumper().toggleOnTrue(m_intake.ingestIntake(false));
    driverController.rightBumper().toggleOnFalse(m_intake.stopIntake());

    driverController.leftBumper().toggleOnTrue(m_intake.ingestIntake(true));
    driverController.leftBumper().toggleOnFalse(m_intake.stopIntake());

    driverController.rightTrigger().onTrue(m_intake.regurgitateIntake());
    driverController.rightTrigger().onFalse(m_intake.stopIntake());

    driverController.b().toggleOnTrue(m_drive.shiftGear());
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
