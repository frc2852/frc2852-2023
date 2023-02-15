package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.libraries.LazySparkMax;
import frc.robot.libraries.SparkMaxFactory;

public class Intake extends SubsystemBase {

    private final LazySparkMax mTop, mRightBottom, mLeftBottom;

    private DigitalInput mIntakeLimitSwitch;

    private static final double MAX_INTAKE_SPEED = 0.5;

    private void configureSpark(LazySparkMax sparkMax, boolean left, boolean master) {
        sparkMax.setInverted(!left);
        sparkMax.enableVoltageCompensation(12.0);
        sparkMax.setOpenLoopRampRate(10);
    }

    public Intake(CommandXboxController driveController) {

        mIntakeLimitSwitch = new DigitalInput(Constants.BOTTOM_INTAKE_LIMIT_SWITCH);

        mTop = SparkMaxFactory.createDefaultSparkMax(Constants.INTAKE_LEFT_TOP);
        configureSpark(mTop, true, true);

        mLeftBottom = SparkMaxFactory.createDefaultSparkMax(Constants.INTAKE_LEFT_BOTTOM);
        configureSpark(mLeftBottom, true, false);

        mRightBottom = SparkMaxFactory.createDefaultSparkMax(Constants.INTAKE_RIGHT_BOTTOM);
        configureSpark(mRightBottom, false, true);
    }

    boolean mIntakeLimitSwitchState = false;

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        mIntakeLimitSwitchState = mIntakeLimitSwitch.get();

        SmartDashboard.putNumber("Motor open loop ramp rate", mLeftBottom.getOpenLoopRampRate());
        SmartDashboard.putNumber("Motor speed", mLeftBottom.get());
        SmartDashboard.putNumber("Motor applied output", mLeftBottom.getAppliedOutput());
        SmartDashboard.putNumber("Motor bus voltage", mLeftBottom.getBusVoltage());
        SmartDashboard.putBoolean("Limit Switch", mIntakeLimitSwitch.get());
    }

    public void ingestIntake() {
        if (!mIntakeLimitSwitchState) {
            this.stopIntake();
            } else {
                //mTop.set(-MAX_INTAKE_SPEED);
                mLeftBottom.set(-MAX_INTAKE_SPEED);
                mRightBottom.set(-MAX_INTAKE_SPEED);
            }
    }

    public void regurgitateIntake() {
        
             if (!mIntakeLimitSwitchState) {
             this.stopIntake();
             } else {
                //mTop.set(MAX_INTAKE_SPEED);
                mLeftBottom.set(MAX_INTAKE_SPEED);
                mRightBottom.set(MAX_INTAKE_SPEED);
             }

    }

    public void stopIntake() {
        stopIntakeBottom();
        stopIntakeTop();
    }

    public void stopIntakeTop() {
        mTop.set(0);
    }

    public void stopIntakeBottom() {
        mLeftBottom.set(0);
        mRightBottom.set(0);
    }
}
