// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Motors;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.MotionMagicVoltage;


public class Splitter extends SubsystemBase {
  /** Creates a new Splitter. */
  private final TalonFX SplitterMotor;
  private final TalonFXConfiguration splitterConfig;
  private double setPointOut;
  private static final double POSITION_TOLERANCE = 0.2;
  private final MotionMagicVoltage motionMagic = new MotionMagicVoltage(0);



  public Splitter(double setPointOut) {
    this.setPointOut = setPointOut;

    /*
     * Initialize the TalonFX motor for the splitter.
     */
    SplitterMotor = new TalonFX(Constants.Motors.SPLITTER_UPTAKE, new CANBus("Indiana"));
    splitterConfig = new TalonFXConfiguration();

    /*
     * Configure the encoder conversion factors if needed.
     */
    splitterConfig.Feedback.SensorToMechanismRatio = 9.0; //9:1 ratio
    splitterConfig.Slot0.kS = 0.25;
    splitterConfig.Slot0.kV = 0.12;
    splitterConfig.Slot0.kA = 0.01;
    splitterConfig.Slot0.kP = 60;
    splitterConfig.Slot0.kI = 0.0;
    splitterConfig.Slot0.kD = 0.5;
    splitterConfig.MotionMagic.MotionMagicCruiseVelocity = 5.0;
    splitterConfig.MotionMagic.MotionMagicAcceleration = 10.0;
    splitterConfig.MotionMagic.MotionMagicJerk = 100.0;
    splitterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;


    SplitterMotor.getConfigurator().apply(splitterConfig);
  }

  public void setSplitterMotorSpeed(double speed) {
    SplitterMotor.set(speed);
  }

  public void stopSplitterMotor() {
    SplitterMotor.set(0);
  }

  public void setSetpoint(double setPoint) {
    this.setPointOut = setPoint;
    SplitterMotor.setControl(motionMagic.withPosition(setPointOut));
  }

  @Override
  public void periodic() {
    // Display encoder positions
    SmartDashboard.putNumber("Splitter Position", SplitterMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Splitter Stator Current", SplitterMotor.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Splitter Supply Current", SplitterMotor.getSupplyCurrent().getValueAsDouble());
  }

}