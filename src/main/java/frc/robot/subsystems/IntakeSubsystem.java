package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

    private final CANSparkMax mRightIntake, mLeftIntake;

    private DoubleSolenoid mIntakeSolenoid;

    private static final double MAX_INTAKE_SPEED = 0.25;
    private static final double MAX_OUTTAKE_SPEED = 1.0;
    private static final double INTAKE_STALL_CURRENT = 24;

    public IntakeSubsystem() {
        mLeftIntake = initMotor(Constants.INTAKE_LEFT, false);
        mRightIntake = initMotor(Constants.INTAKE_RIGHT, true);

        mIntakeSolenoid = new DoubleSolenoid(Constants.PNEUMATIC_HUB, PneumaticsModuleType.REVPH,
                Constants.INTAKE_CLOSE,
                Constants.INTAKE_OPEN);
        setIntakePosition(DoubleSolenoid.Value.kForward);
    }

    public boolean IsIntakeStalled() {
        return (INTAKE_STALL_CURRENT <= mLeftIntake.getOutputCurrent()
                || INTAKE_STALL_CURRENT <= mRightIntake.getOutputCurrent());
    }

    public void ingestIntake(boolean mIsCube) {
        if (mIsCube) {
            setIntakePosition(DoubleSolenoid.Value.kReverse);
            setIntakeSpeeds(-MAX_INTAKE_SPEED);
        } else {
            setIntakePosition(DoubleSolenoid.Value.kForward);
            setIntakeSpeeds(-MAX_INTAKE_SPEED);
        }
    }

    public void regurgitateIntake() {
        setIntakeSpeeds(MAX_OUTTAKE_SPEED);
    }

    public void stopIntake() {
        setIntakeSpeeds(0);
    }

    private void setIntakeSpeeds(double motorSpeed) {
        mLeftIntake.set(motorSpeed);
        mRightIntake.set(motorSpeed);
    }

    public void setIntakePosition(Value solenoidPosition) {
        mIntakeSolenoid.set(solenoidPosition);
    }

    private CANSparkMax initMotor(int deviceId, boolean isInverted) {
        CANSparkMax motor = new CANSparkMax(deviceId, MotorType.kBrushless);
        motor.restoreFactoryDefaults();
        motor.setInverted(isInverted);
        motor.enableVoltageCompensation(12.0);
        motor.setSmartCurrentLimit(20);
        motor.setIdleMode(IdleMode.kBrake);

        // Minimimize CAN bus usage
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);

        motor.burnFlash();
        return motor;
    }
}
