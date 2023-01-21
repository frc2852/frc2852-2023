package frc.robot.subsystems;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.libraries.LazySparkMax;
import frc.robot.libraries.SparkMaxFactory;

public class Intake extends SubsystemBase {
  
//   private final LazySparkMax mLeftMaster, mRightMaster; //mLeftSlave, mRightSlave;
//   private DifferentialDrive mDifferentialDrive;
//   private CommandXboxController driverController;

//   private static final double MAX_INTAKE_SPEED = 0.1;

//   private void configureSpark(LazySparkMax sparkMax, boolean left, boolean master) {
//     sparkMax.setInverted(!left);
//     sparkMax.enableVoltageCompensation(12.0);
//     sparkMax.setClosedLoopRampRate(Constants.DRIVE_VOLTAGE_RAMP_RATE);

// }
// public Intake(CommandXboxController driveController) {

//     mLeftMaster = SparkMaxFactory.createDefaultSparkMax(Constants.INTAKE_LEFT_MASTER);
//     configureSpark(mLeftMaster, true, true);

//     //mLeftSlave = SparkMaxFactory.createPermanentSlaveSparkMax(Constants.INTAKE_LEFT_SLAVE, mLeftMaster);
//     //configureSpark(mLeftSlave, true, false);

//     mRightMaster = SparkMaxFactory.createDefaultSparkMax(Constants.INTAKE_RIGHT_MASTER);
//     configureSpark(mRightMaster, false, true);

//     //mRightSlave = SparkMaxFactory.createPermanentSlaveSparkMax(Constants.INTAKE_RIGHT_SLAVE, mRightMaster);
//     //configureSpark(mRightSlave, false, false);

//     mDifferentialDrive = new DifferentialDrive(mLeftMaster, mRightMaster);

//     this.driverController = driveController;
// }

// @Override
// public void periodic() {
//   // This method will be called once per scheduler run

// }

// public CommandBase ingestIntake(){
//     return run(() -> {
//         mDifferentialDrive.tankDrive(MAX_INTAKE_SPEED, MAX_INTAKE_SPEED);
//     });
    
// }
// public CommandBase regurgitateIntake(){
//     return run(() -> {
//         mDifferentialDrive.tankDrive(-MAX_INTAKE_SPEED, -MAX_INTAKE_SPEED);
//     });
// }

}