// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.libraries;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;

public class SparkMaxExtended extends CANSparkMax {

    protected double mLastSet = Double.NaN;
    protected ControlType mLastControlType = null;
    protected CANSparkMax mLeader = null;

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
}
