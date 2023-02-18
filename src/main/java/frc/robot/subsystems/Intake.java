package frc.robot.subsystems;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.libraries.LazySparkMax;
import frc.robot.libraries.SparkMaxFactory;

public class Intake extends SubsystemBase {

    private final LazySparkMax mTop, mRightBottom, mLeftBottom;

    private DigitalInput mIntakeLimitSwitch;
    private DoubleSolenoid mIntakeSolenoid;
    private boolean mIntakeIsInCubeState = false;

    private static final double MAX_INTAKE_SPEED = 0.3;
    private static final double RAMP_RATE = 10;

    private void configureSpark(LazySparkMax sparkMax, boolean invert) {
        sparkMax.setInverted(invert);
        sparkMax.setIdleMode(IdleMode.kBrake);
        sparkMax.enableVoltageCompensation(12.0);
        sparkMax.setOpenLoopRampRate(RAMP_RATE);
        sparkMax.burnFlash();
    }

    public Intake(CommandXboxController driveController) {

        mIntakeLimitSwitch = new DigitalInput(Constants.BOTTOM_INTAKE_LIMIT_SWITCH);

        mTop = SparkMaxFactory.createDefaultSparkMax(Constants.INTAKE_LEFT_TOP);
        configureSpark(mTop, false);

        mLeftBottom = SparkMaxFactory.createDefaultSparkMax(Constants.INTAKE_LEFT_BOTTOM);
        configureSpark(mLeftBottom, false);

        mRightBottom = SparkMaxFactory.createDefaultSparkMax(Constants.INTAKE_RIGHT_BOTTOM);
        configureSpark(mRightBottom, true);

        mIntakeSolenoid = new DoubleSolenoid(Constants.PNEUMATIC_HUB, PneumaticsModuleType.REVPH, Constants.INTAKE_OPEN,
                Constants.INTAKE_OPEN);
        setDefaultPosition(DoubleSolenoid.Value.kForward);
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

    public CommandBase ingestIntake(boolean mIsCube) {
        return run(() -> {
            if (mIsCube) {
                setDefaultPosition(DoubleSolenoid.Value.kReverse);
            } else {
                setDefaultPosition(DoubleSolenoid.Value.kForward);
            }

            if (!mIntakeLimitSwitchState) {
                this.stopIntake();
            } else {
                setIntakeSpeeds(-MAX_INTAKE_SPEED, RAMP_RATE);
            }
        });
    }

    public CommandBase regurgitateIntake() {
        return run(() -> {
            setIntakeSpeeds(MAX_INTAKE_SPEED, RAMP_RATE);
        });
    }

    public CommandBase stopIntake() {
        return run(() -> {
            setIntakeSpeeds(0, 0);
        });
    }

    public void setIntakeSpeeds(double motorSpeed, double rampRate) {
        mTop.setOpenLoopRampRate(rampRate);
        mRightBottom.setOpenLoopRampRate(rampRate);
        mLeftBottom.setOpenLoopRampRate(rampRate);

        mTop.set(motorSpeed);
        mLeftBottom.set(motorSpeed);
        mRightBottom.set(motorSpeed);
    }

    public void setDefaultPosition(Value solenoidPosition) {
        mIntakeIsInCubeState = (solenoidPosition == DoubleSolenoid.Value.kReverse);
        mIntakeSolenoid.set(solenoidPosition);
    }
}
