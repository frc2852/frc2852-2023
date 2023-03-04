// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Arm extends SubsystemBase {

  private final double kP = 0.1; // Proportional constant for PID control
  private final double kI = 0.0; // Integral constant for PID control
  private final double kD = 0.0; // Derivative constant for PID control

  private final CANSparkMax wristMotor = new CANSparkMax(55, MotorType.kBrushless);
  private final Encoder wristEncoder = new Encoder(0, 1, false, CounterBase.EncodingType.k2X);

  private final PIDController wristPIDController = new PIDController(kP, kI, kD);

  public Arm() {
    // Initialize the arm motor
    wristMotor.setInverted(true);

    wristEncoder.setSamplesToAverage(5);
    wristEncoder.setDistancePerPulse(4.0 / 8192);
    wristEncoder.setMinRate(1.0);
    // wristEncoder.reset();

    // Initialize the PID controller for the arm
    wristPIDController.setTolerance(1.0);
    wristPIDController.setSetpoint(90);
    wristPIDController.setIntegratorRange(-0.1, 0.1);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Encoder Distance 2", wristEncoder.getDistance());
    SmartDashboard.putNumber("Encoder Rate 2", wristEncoder.getRate());

    double output = wristPIDController.calculate(wristEncoder.getDistance());
    wristMotor.set(output);
    SmartDashboard.putNumber("Output", output);
  }
}
