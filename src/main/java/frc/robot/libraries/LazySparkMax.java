// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.libraries;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;

public class LazySparkMax extends CANSparkMax {

    protected double mLastSet = Double.NaN;
    protected ControlType mLastControlType = null;

    // Set if is a follower
    protected CANSparkMax mLeader = null;

    public LazySparkMax(int deviceId) {
        super(deviceId, MotorType.kBrushless);
    }

    public CANSparkMax getLeader() {
        return mLeader;
    }

    @Override
    public REVLibError follow(final CANSparkMax leader) {
        mLeader = leader;
        return super.follow(leader);
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
}
