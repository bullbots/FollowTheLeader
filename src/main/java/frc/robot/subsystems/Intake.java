// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.FrontIntake;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Intake extends SubsystemBase {
  public static final String setIntakeMotorSpeed = null;
  /** Creates a new Intake. */
  private final TalonFX intakeMotor;
  public Intake() {
    intakeMotor = new TalonFX(Constants.Motors.INTAKE, new CANBus("Indiana"));

    // Configure current limit
    TalonFXConfiguration config = new TalonFXConfiguration();
    CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
    currentLimits.SupplyCurrentLimit = 40;
    currentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits = currentLimits;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    intakeMotor.getConfigurator().apply(config);
  }

  public void setIntakeMotorSpeed() {
    intakeMotor.set(-0.5);
  }

  public void stopIntakeMotor() {
    intakeMotor.set(0);
  }

  public void lawnMowerMode() {
    intakeMotor.set(-0.95);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
