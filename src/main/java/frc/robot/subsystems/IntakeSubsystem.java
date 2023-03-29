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

    private static final double MAX_INTAKE_SPEED = 1.0;
    private static final double MAX_OUTTAKE_SPEED = 1.0;
    private static final double INTAKE_STALL_CURRENT = 24;

    public IntakeSubsystem() {
        mLeftIntake = new CANSparkMax(Constants.INTAKE_LEFT_BOTTOM, MotorType.kBrushless);
        mLeftIntake.setInverted(true);
        mLeftIntake.setIdleMode(IdleMode.kBrake);
        mLeftIntake.enableVoltageCompensation(12.0);
        mLeftIntake.burnFlash();

        mLeftIntake.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
        mLeftIntake.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
        mLeftIntake.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);

        mRightIntake = new CANSparkMax(Constants.INTAKE_RIGHT_BOTTOM, MotorType.kBrushless);
        mRightIntake.setInverted(false);
        mRightIntake.setIdleMode(IdleMode.kBrake);
        mRightIntake.enableVoltageCompensation(12.0);
        mRightIntake.burnFlash();

        mRightIntake.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
        mRightIntake.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
        mRightIntake.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);
        
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
}
