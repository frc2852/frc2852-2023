// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ArmPosition;
import frc.robot.commands.arm.DrivePosition;
import frc.robot.commands.arm.HighPickupPosition;
import frc.robot.commands.arm.ScorePositionLow;
import frc.robot.commands.arm.ScorePositionMidCube;
import frc.robot.commands.arm.ScorePositionMidPylon;
import frc.robot.commands.arm.PickupPosition;
import frc.robot.commands.arm.ScorePositionHighCube;
import frc.robot.commands.arm.ZeroPosition;
import frc.robot.commands.arm.manual.ManualInnerPosition;
import frc.robot.commands.arm.manual.ManualWristPosition;
import frc.robot.commands.autos.HighScoreCubeDriveForwardAuto;
import frc.robot.commands.autos.MidScoreCubeDriveForwardAuto;
import frc.robot.commands.autos.MidScorePylonDriveForwardAuto;
import frc.robot.commands.autos.HighScoreCubeBalanceAuto;
import frc.robot.commands.drive.Drive;
import frc.robot.commands.drive.ToggleGear;
import frc.robot.commands.intake.IntakeCube;
import frc.robot.commands.intake.IntakePylon;
import frc.robot.commands.intake.Outtake;
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
  private final IntakeCube mIntakeCubeCmd = new IntakeCube(mIntakeSubsystem);
  private final IntakePylon mIntakePylonCmd = new IntakePylon(mIntakeSubsystem);
  private final Outtake mOuttakeCmd = new Outtake(mIntakeSubsystem);
  private final DrivePosition mDrivePositionCmd = new DrivePosition(mArmSubsystem);
  private final ToggleGear mToggleGearCmd = new ToggleGear(mDriveSubsystem);

  // Operator commands
  private final ZeroPosition mZeroPositionCmd = new ZeroPosition(mArmSubsystem);
  private final PickupPosition mPickupPositionCmd = new PickupPosition(mArmSubsystem);
  private final HighPickupPosition mHighPickupPositionCmd = new HighPickupPosition(mArmSubsystem);

  private final ScorePositionLow mScorePositionLowCmd = new ScorePositionLow(mArmSubsystem);
  private final ScorePositionMidPylon mScorePositionMidPylonCmd = new ScorePositionMidPylon(mArmSubsystem);
  private final ScorePositionMidCube mScorePositionMidCubeCmd = new ScorePositionMidCube(mArmSubsystem);
  private final ScorePositionHighCube mScorePositionHighCubeCmd = new ScorePositionHighCube(mArmSubsystem);

  private final ManualWristPosition mManualWristBack = new ManualWristPosition(false);
  private final ManualWristPosition mManualWristForward = new ManualWristPosition(true);

  private final ManualInnerPosition mManualInnerBack = new ManualInnerPosition(false);
  private final ManualInnerPosition mManualInnerForward = new ManualInnerPosition(true);

  // Autos
  private final SendableChooser<Command> autoSelection = new SendableChooser<>();
  private final HighScoreCubeDriveForwardAuto mHighScoreCubeDriveForwardAuto = new HighScoreCubeDriveForwardAuto(
      mDriveSubsystem, mArmSubsystem, mIntakeSubsystem);
  private final MidScoreCubeDriveForwardAuto mMidScoreCubeDriveForwardAuto = new MidScoreCubeDriveForwardAuto(
      mDriveSubsystem, mArmSubsystem, mIntakeSubsystem);
  private final MidScorePylonDriveForwardAuto mMidScorePylonDriveForwardAuto = new MidScorePylonDriveForwardAuto(
      mDriveSubsystem, mArmSubsystem, mIntakeSubsystem);
  private final HighScoreCubeBalanceAuto mHighScoreCubeBalanceAuto = new HighScoreCubeBalanceAuto(mDriveSubsystem,
      mArmSubsystem, mIntakeSubsystem);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    ArmSubsystem.ARM_POSITION = ArmPosition.ZERO;

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
        new Drive(mDriveSubsystem, () -> driverController.getLeftX(), () -> driverController.getLeftY()));

    driverController.a().onTrue(mDrivePositionCmd);
    driverController.b().onTrue(mToggleGearCmd);

    driverController.rightBumper().whileTrue(mIntakeCubeCmd);
    driverController.leftBumper().whileTrue(mIntakePylonCmd);
    driverController.rightTrigger().whileTrue(mOuttakeCmd);
  }

  private void configureOperatorController() {
    operatorController.a().onTrue(mScorePositionLowCmd);
    operatorController.b().onTrue(mScorePositionMidPylonCmd); // Mid score position pylon
    operatorController.x().onTrue(mScorePositionMidCubeCmd); // Mid score position cube
    operatorController.y().onTrue(mScorePositionHighCubeCmd); // High score position cube
    operatorController.leftBumper().onTrue(mHighPickupPositionCmd); // Pick up high position
    operatorController.rightBumper().onTrue(mPickupPositionCmd); // Pick up low position

    operatorController.povUp().onTrue(mManualInnerForward);
    operatorController.povDown().onTrue(mManualInnerBack);
    operatorController.povLeft().onTrue(mManualWristForward);
    operatorController.povRight().onTrue(mManualWristBack);
    // Secret
    operatorController.back().onTrue(mZeroPositionCmd);
  }

  private void configureAutoSelection() {
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
