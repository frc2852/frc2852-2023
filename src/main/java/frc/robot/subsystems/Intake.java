package frc.robot.subsystems;

import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
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
  
   private final LazySparkMax mLeftTop, mRightBottom, mLeftBottom;
   private CommandXboxController driverController;

  // private DigitalInput mBottomIntakeLimitSwitch = new DigitalInput(Constants.BOTTOM_INTAKE_LIMIT_SWITCH);
  // private DigitalInput mTopIntakeLimitSwitch = new DigitalInput(Constants.TOP_INTAKE_LIMIT_SWITCH);

   private static final double MAX_INTAKE_SPEED = 0.2;

   private void configureSpark(LazySparkMax sparkMax, boolean left, boolean master) {
     sparkMax.setInverted(!left);
     sparkMax.enableVoltageCompensation(12.0);
     sparkMax.setClosedLoopRampRate(0.0);

 }
 public Intake(CommandXboxController driveController) {

     mLeftTop = SparkMaxFactory.createDefaultSparkMax(Constants.INTAKE_LEFT_TOP);
     configureSpark(mLeftTop, true, true);

     mLeftBottom = SparkMaxFactory.createDefaultSparkMax(Constants.INTAKE_LEFT_BOTTOM);
     configureSpark(mLeftBottom, true, false);

     mRightBottom = SparkMaxFactory.createDefaultSparkMax(Constants.INTAKE_RIGHT_BOTTOM);
     configureSpark(mRightBottom, false, true);

     

     this.driverController = driveController;
 }

 @Override
 public void periodic() {
    //This method will be called once per scheduler run

}

 public CommandBase ingestIntake(){
     return runOnce(() -> {
       /*  if (mTopIntakeLimitSwitch.get()) {
            this.stopIntakeTop();
        } else {
            mLeftTop.set(-MAX_INTAKE_SPEED);
        }
       if (mBottomIntakeLimitSwitch.get()) {
        this.stopIntakeBottom();
       } else {
            mLeftBottom.set(-MAX_INTAKE_SPEED);
            mRightBottom.set(-MAX_INTAKE_SPEED);
       }*/
    //    mLeftBottom.setClosedLoopRampRate(20.0);
    //    mRightBottom.setClosedLoopRampRate(20.0);
       
       mLeftBottom.set(-MAX_INTAKE_SPEED);
       mRightBottom.set(-MAX_INTAKE_SPEED);
       mLeftTop.set(-MAX_INTAKE_SPEED);
    });
 }
 public CommandBase regurgitateIntake(){
     return runOnce(() -> {
        /*if (mTopIntakeLimitSwitch.get()) {
            this.stopIntakeTop();
        } else {
            mLeftTop.set(MAX_INTAKE_SPEED);
        }
        if (mBottomIntakeLimitSwitch.get()) {
            this.stopIntake();
        } else {
            mLeftBottom.set(MAX_INTAKE_SPEED);
            mRightBottom.set(MAX_INTAKE_SPEED); 
        }*/
        mLeftBottom.set(MAX_INTAKE_SPEED);
        mLeftTop.set(MAX_INTAKE_SPEED);
        mRightBottom.set(MAX_INTAKE_SPEED); 
    });
 }
    public CommandBase stopIntake(){ 
        return run(() -> {
        stopIntakeBottom();
        stopIntakeTop();
      });
    }
    public void stopIntakeTop(){
        mLeftTop.set(0);
 }
    public void stopIntakeBottom(){
        mLeftBottom.set(0);
        mRightBottom.set(0);
 }

}

