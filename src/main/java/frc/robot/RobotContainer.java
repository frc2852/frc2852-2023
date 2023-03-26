// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ArmPosition;
import frc.robot.commands.Arm.DrivePositionCommand;
import frc.robot.commands.Arm.HighPickupPositionCommand;
import frc.robot.commands.Arm.ScorePositionLowCommand;
import frc.robot.commands.Arm.ScorePositionMidCubeCommand;
import frc.robot.commands.Arm.ScorePositionMidPylonCommand;
import frc.robot.commands.Arm.PickupPositionCommand;
import frc.robot.commands.Arm.ScorePositionHighCubeCommand;
import frc.robot.commands.Arm.ZeroPositionCommand;
import frc.robot.commands.Arm.Manual.ManualInner;
import frc.robot.commands.Arm.Manual.ManualWrist;
import frc.robot.commands.autos.HighScoreCubeDriveForwardAuto;
import frc.robot.commands.autos.MidScoreCubeDriveForwardAuto;
import frc.robot.commands.autos.MidScorePylonDriveForwardAuto;
import frc.robot.commands.autos.HighScoreCubeBalanceAuto;
import frc.robot.commands.drive.DriveCommand;
import frc.robot.commands.drive.ToggleGear;
import frc.robot.commands.Intake.IntakeCubeCommand;
import frc.robot.commands.Intake.IntakePylonCommand;
import frc.robot.commands.Intake.OuttakeCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  private final ScorePositionMidPylonCommand mScorePositionMidPylonCommand = new ScorePositionMidPylonCommand(mArmSubsystem);
  private final ScorePositionMidCubeCommand mScorePositionMidCubeCommand = new ScorePositionMidCubeCommand(mArmSubsystem);
  private final ScorePositionHighCubeCommand mScorePositionHighCubeCommand = new ScorePositionHighCubeCommand(mArmSubsystem);

  private final ManualWrist mManualWristBack = new ManualWrist(false);
  private final ManualWrist mManualWristForward = new ManualWrist(true);

  private final ManualInner mManualInnerBack = new ManualInner(false);
  private final ManualInner mManualInnerForward = new ManualInner(true);

  //Autos
  private final SendableChooser<Command> autoSelection = new SendableChooser<>();
  private final HighScoreCubeDriveForwardAuto mHighScoreCubeDriveForwardAuto = new HighScoreCubeDriveForwardAuto(mDriveSubsystem, mArmSubsystem, mIntakeSubsystem);
  private final MidScoreCubeDriveForwardAuto mMidScoreCubeDriveForwardAuto = new MidScoreCubeDriveForwardAuto(mDriveSubsystem, mArmSubsystem, mIntakeSubsystem);  
  private final MidScorePylonDriveForwardAuto mMidScorePylonDriveForwardAuto = new MidScorePylonDriveForwardAuto(mDriveSubsystem, mArmSubsystem, mIntakeSubsystem);
  private final HighScoreCubeBalanceAuto mHighScoreCubeBalanceAuto = new HighScoreCubeBalanceAuto(mDriveSubsystem, mArmSubsystem, mIntakeSubsystem);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    ArmSubsystem.armPosition = ArmPosition.ZERO;

    // Configure the trigger bindings
    configureDriveController();
    configureOperatorController();
    configureAutoSelection();

    mPneumaticHub.enableCompressorDigital();

    CameraServer.startAutomaticCapture();
    CameraServer.startAutomaticCapture();
  }

  private void configureDriveController() {
    mDriveSubsystem.setDefaultCommand(
        new DriveCommand(mDriveSubsystem, () -> driverController.getLeftX(), () -> driverController.getLeftY()));

    driverController.a().onTrue(mDrivePositionCommand);
    driverController.b().onTrue(mToggleGearCommand);

    driverController.rightBumper().whileTrue(mIntakeCubeCommand);
    driverController.leftBumper().whileTrue(mIntakePylonCommand);
    driverController.rightTrigger().whileTrue(mOuttakeCommand);
  }

  private void configureOperatorController() {
    operatorController.a().onTrue(mScorePositionLowCommand);
    operatorController.b().onTrue(mScorePositionMidPylonCommand); //Mid score position pylon
    operatorController.x().onTrue(mScorePositionMidCubeCommand); //Mid score position cube
    operatorController.y().onTrue(mScorePositionHighCubeCommand); //High score position cube
    operatorController.leftBumper().onTrue(mHighPickupPositionCommand); //Pick up high position
    operatorController.rightBumper().onTrue(mPickupPositionCommand); //Pick up low position

    operatorController.povUp().onTrue(mManualInnerForward);
    operatorController.povDown().onTrue(mManualInnerBack);
    operatorController.povLeft().onTrue(mManualWristForward);
    operatorController.povRight().onTrue(mManualWristBack);
    //Secret
    operatorController.back().onTrue(mZeroPositionCommand);
  }

  private void configureAutoSelection(){
    autoSelection.setDefaultOption("High score cube - Drive forward", mHighScoreCubeDriveForwardAuto);
    autoSelection.addOption("Mid score cube - Drive forward", mMidScoreCubeDriveForwardAuto);
    autoSelection.addOption("Mid score pylon - Drive forward", mMidScorePylonDriveForwardAuto);
    autoSelection.addOption("High Score Cube - Then RAMP", mHighScoreCubeBalanceAuto);

    SmartDashboard.putData("Auto Mode", autoSelection);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoSelection.getSelected();
  }
}
