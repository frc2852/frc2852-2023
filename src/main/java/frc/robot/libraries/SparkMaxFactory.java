// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.libraries;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

/**
 * Thanks Cheesy Poofs!
 * https://github.com/Team254/FRC-2019-Public
 */
public class SparkMaxFactory {
    public static class Configuration {
        public boolean BURN_FACTORY_DEFAULT_FLASH = false;
        public IdleMode NEUTRAL_MODE = IdleMode.kCoast;
        public boolean INVERTED = false;

        public int STATUS_FRAME_0_RATE_MS = 10;
        public int STATUS_FRAME_1_RATE_MS = 1000;
        public int STATUS_FRAME_2_RATE_MS = 1000;

        public double OPEN_LOOP_RAMP_RATE = 0.0;
        public double CLOSED_LOOP_RAMP_RATE = 0.0;

        public boolean ENABLE_VOLTAGE_COMPENSATION = false;
        public double NOMINAL_VOLTAGE = 12.0;
    }

    private static final Configuration kDefaultConfiguration = new Configuration();
    private static final Configuration kSlaveConfiguration = new Configuration();

    static {
        kSlaveConfiguration.STATUS_FRAME_0_RATE_MS = 1000;
        kSlaveConfiguration.STATUS_FRAME_1_RATE_MS = 1000;
        kSlaveConfiguration.STATUS_FRAME_2_RATE_MS = 1000;
    }

    // Create a CANTalon with the default (out of the box) configuration.
    public static LazySparkMax createDefaultSparkMax(int id) {
        return createSparkMax(id, kDefaultConfiguration);
    }

    private static void handleREVLibError(int id, REVLibError error, String message) {
        DriverStation.reportError("Could not configure spark id: " + id + " error: " + error.toString() + " " + message, false);
    }

    public static LazySparkMax createPermanentSlaveSparkMax(int id, CANSparkMax master) {
        final LazySparkMax sparkMax = createSparkMax(id, kSlaveConfiguration);
        handleREVLibError(id, sparkMax.follow(master), "setting follower");
        return sparkMax;
    }

    public static LazySparkMax createSparkMax(int id, Configuration config) {
        // Delay for CAN bus bandwidth to clear up.
        Timer.delay(0.25);
        LazySparkMax sparkMax = new LazySparkMax(id);
        handleREVLibError(id, sparkMax.setCANTimeout(200), "set timeout");

        // sparkMax.restoreFactoryDefaults(config.BURN_FACTORY_DEFAULT_FLASH);

        sparkMax.set(ControlType.kDutyCycle, 0.0);

        handleREVLibError(id, sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus0, config.STATUS_FRAME_0_RATE_MS), "set status0 rate");
        handleREVLibError(id, sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus1, config.STATUS_FRAME_1_RATE_MS), "set status1 rate");
        handleREVLibError(id, sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus2, config.STATUS_FRAME_2_RATE_MS), "set status2 rate");

        sparkMax.clearFaults();

        handleREVLibError(id, sparkMax.setIdleMode(config.NEUTRAL_MODE), "set neutrual");
        sparkMax.setInverted(config.INVERTED);
        handleREVLibError(id, sparkMax.setOpenLoopRampRate(config.OPEN_LOOP_RAMP_RATE), "set open loop ramp");
        handleREVLibError(id, sparkMax.setClosedLoopRampRate(config.CLOSED_LOOP_RAMP_RATE), "set closed loop ramp");

        if (config.ENABLE_VOLTAGE_COMPENSATION) {
            handleREVLibError(id, sparkMax.enableVoltageCompensation(config.NOMINAL_VOLTAGE), "voltage compensation");
        } else {
            handleREVLibError(id, sparkMax.disableVoltageCompensation(), "voltage compensation");
        }

        return sparkMax;
    }
}
