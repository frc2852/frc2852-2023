// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Arm.DrivePositionCommand;
import frc.robot.commands.Arm.HighPickupPositionCommand;
import frc.robot.commands.Arm.ScorePositionLowCommand;
import frc.robot.commands.Arm.ScorePositionMidCommand;
import frc.robot.commands.Arm.PickupPositionCommand;
import frc.robot.commands.Arm.ScorePositionHighCommand;
import frc.robot.commands.Arm.ZeroPositionCommand;
import frc.robot.commands.Autos.LowScoreDriveForwardAuto;
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
import edu.wpi.first.wpilibj2.command.CommandScheduler;
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

  private final PneumaticHub mPneumaticHub = new PneumaticHub(Constants.PNEUMATIC_HUB);

  // Driver commands
  private final IntakeCubeCommand mIntakeCubeCommand = new IntakeCubeCommand(mIntakeSubsystem);
  private final IntakePylonCommand mIntakePylonCommand = new IntakePylonCommand(mIntakeSubsystem);
  private final OuttakeCommand mOuttakeCommand = new OuttakeCommand(mIntakeSubsystem);
  private final DrivePositionCommand mDrivePositionCommand = new DrivePositionCommand(mArmSubsystem);
  private final ToggleGear mToggleGearCommand = new ToggleGear(mDriveSubsystem);
  
  // Operator commands
  private final ZeroPositionCommand mZeroPositionCommand = new ZeroPositionCommand(mArmSubsystem);
  private final PickupPositionCommand mPickupPositionCommand = new PickupPositionCommand(mArmSubsystem);
  private final HighPickupPositionCommand mHighPickupPositionCommand = new HighPickupPositionCommand(mArmSubsystem);

  private final ScorePositionLowCommand mScorePositionLowCommand = new ScorePositionLowCommand(mArmSubsystem);
  private final ScorePositionMidCommand mScorePositionMidCommand = new ScorePositionMidCommand(mArmSubsystem);
  //private final ScorePositionHighCommand mScorePositionHighCommand = new ScorePositionHighCommand(mArmSubsystem);

  //Autos
  private final LowScoreDriveForwardAuto mLowScoreDriveForwardAuto = new LowScoreDriveForwardAuto(mArmSubsystem, mIntakeSubsystem);  

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    ConfigureDriveController();
    ConfigureOperatorController();
    CommandScheduler.getInstance().schedule(mDrivePositionCommand);
    mPneumaticHub.enableCompressorDigital();
  }

  private void ConfigureDriveController() {
    mDriveSubsystem.setDefaultCommand(
        new DriveCommand(mDriveSubsystem, () -> driverController.getLeftX(), () -> driverController.getLeftY()));

    driverController.a().onTrue(mDrivePositionCommand);
    driverController.b().onTrue(mToggleGearCommand);

    driverController.rightBumper().whileTrue(mIntakeCubeCommand);
    driverController.leftBumper().whileTrue(mIntakePylonCommand);
    driverController.rightTrigger().whileTrue(mOuttakeCommand);
  }

  private void ConfigureOperatorController() {
    operatorController.x().onTrue(mPickupPositionCommand);
    operatorController.y().onTrue(mHighPickupPositionCommand);
    operatorController.a().onTrue(mScorePositionLowCommand);
    operatorController.b().onTrue(mScorePositionMidCommand);
    //operatorController.y().onTrue(mScorePositionHighCommand);
    operatorController.back().onTrue(mZeroPositionCommand);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return mLowScoreDriveForwardAuto;
  }
}
