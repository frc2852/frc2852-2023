// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.libraries;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SparkMaxExtended extends CANSparkMax {

    protected double mLastSet = Double.NaN;
    protected ControlType mLastControlType = null;

    // Set if is a follower
    protected CANSparkMax mLeader = null;

    public Encoder encoder = null;
    public PIDController pidController;
    private double mSetPoint = 0;

    public int INDUSTRIAL_ENCODER_TICKS = 63;

    public SparkMaxExtended(int deviceId) {
        super(deviceId, MotorType.kBrushless);
    }

    public SparkMaxExtended(int deviceId, SparkMaxExtended leader) {
        super(deviceId, MotorType.kBrushless);
        this.follow(leader);
    }

    public CANSparkMax getLeader() {
        return mLeader;
    }

    @Override
    public REVLibError follow(final CANSparkMax leader) {
        mLeader = leader;
        return super.follow(leader);
    }

    public void setFeedbackDevice(int channelA, int channelB, double kp, double ki, double kd) {
        // Configure encoder
        encoder = new Encoder(channelA, channelB, false, EncodingType.k2X);
        encoder.setDistancePerPulse(1.0 / INDUSTRIAL_ENCODER_TICKS);
        encoder.setReverseDirection(true);
        // Configure spark max
        super.getEncoder().setPositionConversionFactor(1.0 / INDUSTRIAL_ENCODER_TICKS);
        super.getEncoder().setPosition(0); // initial position

        // Configure pid controller
        pidController = new PIDController(kp, ki, kd);
        pidController.setSetpoint(0.0);
        pidController.setTolerance(1.0 / INDUSTRIAL_ENCODER_TICKS);
    }

    /**
     * wrapper method to mimic TalonSRX set method
     */
    public void set(ControlType type, double setpoint) {
        if (setpoint != mLastSet || type != mLastControlType) {
            mLastSet = setpoint;
            mLastControlType = type;
            super.getPIDController().setReference(setpoint, type);
        }
    }

    public void setSetPoint(double setPoint) {
        mSetPoint = setPoint;
        pidController.setSetpoint(mSetPoint);
    }

    public void periodic() {
        // Read the encoder count and convert to radians
        double position = Math.toRadians(encoder.getDistance());

        // Set the setpoint for the PID controller

        // Get the output from the PID controller
        double output = pidController.calculate(position);

        // Set the motor output
        super.set(output);

        SmartDashboardUpdate();
    }

    private void SmartDashboardUpdate() {
        SmartDashboard.putNumber("Encoder decoding scale", encoder.getDecodingScaleFactor());
        SmartDashboard.putNumber("Encoder distance", encoder.getDistance());
        SmartDashboard.putNumber("Encoder distance per pulse", encoder.getDistancePerPulse());
        SmartDashboard.putNumber("Encoder rate", encoder.getRate());
        SmartDashboard.putNumber("Encoder raw", encoder.getRaw());
        SmartDashboard.putNumber("Set Point", pidController.getSetpoint());
    }
}
