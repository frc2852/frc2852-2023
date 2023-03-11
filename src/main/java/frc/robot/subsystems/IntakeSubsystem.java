package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

    private final CANSparkMax mRightIntake, mLeftIntake;

    private DoubleSolenoid mIntakeSolenoid;
    private double leftMaxCurrent = 0;
    private double rightMaxCurrent = 0;

    private static final double MAX_INTAKE_SPEED = 0.20;
    private static final double INTAKE_STALL_CURRENT = 28;

    private void configureSpark(CANSparkMax sparkMax, boolean invert) {
        sparkMax.setInverted(invert);
        sparkMax.setIdleMode(IdleMode.kCoast);
        // sparkMax.enableVoltageCompensation(12.0);
        sparkMax.burnFlash();
    }

    public IntakeSubsystem() {
        mLeftIntake = new CANSparkMax(Constants.INTAKE_LEFT_BOTTOM, MotorType.kBrushless);
        configureSpark(mLeftIntake, false);

        mRightIntake = new CANSparkMax(Constants.INTAKE_RIGHT_BOTTOM, MotorType.kBrushless);
        configureSpark(mRightIntake, true);

        mIntakeSolenoid = new DoubleSolenoid(Constants.PNEUMATIC_HUB, PneumaticsModuleType.REVPH,
                Constants.INTAKE_CLOSE,
                Constants.INTAKE_OPEN);
        setIntakePosition(DoubleSolenoid.Value.kForward);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        double currentLeftCurrent = mLeftIntake.getOutputCurrent();
        if (leftMaxCurrent < currentLeftCurrent) {
            leftMaxCurrent = currentLeftCurrent;
        }

        double currentRightCurrent = mRightIntake.getOutputCurrent();
        if (rightMaxCurrent < currentRightCurrent) {
            rightMaxCurrent = currentRightCurrent;
        }

        SmartDashboard.putNumber("Left Intake current", leftMaxCurrent);
        SmartDashboard.putNumber("Right Intake current", rightMaxCurrent);
    }

    public boolean IsIntakeStalled() {
        return (INTAKE_STALL_CURRENT <= mLeftIntake.getOutputCurrent() || INTAKE_STALL_CURRENT <= mRightIntake.getOutputCurrent());
    }

    public void ingestIntake(boolean mIsCube) {
        if (mIsCube) {
            setIntakePosition(DoubleSolenoid.Value.kReverse);
            setIntakeSpeeds(-MAX_INTAKE_SPEED, 0.0);
        } else {
            setIntakePosition(DoubleSolenoid.Value.kForward);
            setIntakeSpeeds(-0.4, 0.0);
        }
    }

    public void regurgitateIntake() {
        setIntakeSpeeds(MAX_INTAKE_SPEED, 0.0);
    }

    public void stopIntake() {
        setIntakeSpeeds(0, 0);
    }

    private void setIntakeSpeeds(double motorSpeed, double rampRate) {
        mLeftIntake.set(motorSpeed);
        mRightIntake.set(motorSpeed);
    }

    private void setIntakePosition(Value solenoidPosition) {
        mIntakeSolenoid.set(solenoidPosition);
    }
}
