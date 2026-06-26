// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.RelativeEncoder;



public class Rollers extends SubsystemBase {
  /** Creates a new Rollers. */
  private final SparkMax rollerMotor;
  private final SparkMaxConfig rollerMotorConfig;
  private final SparkClosedLoopController closedLoopControllerRoller;
  private final RelativeEncoder encoderRoller;

  public Rollers() {
    rollerMotor = new SparkMax(Constants.Motors.ROLLERS, MotorType.kBrushless);
    closedLoopControllerRoller = rollerMotor.getClosedLoopController();
    encoderRoller = rollerMotor.getEncoder();
    rollerMotorConfig = new SparkMaxConfig();
    rollerMotorConfig.idleMode(SparkMaxConfig.IdleMode.kBrake);

    // Current limiting configuration for motor safety
    rollerMotorConfig
      .smartCurrentLimit(40); // Limit current to 40A for the roller motor

    rollerMotor.configure(rollerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public void moveRollers (){
    rollerMotor.set(1);
  }

  public void moveRollersAtSpeed(double speed) {
    rollerMotor.set(speed);
  }

  public void stopRollers() {
    rollerMotor.set(0);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
