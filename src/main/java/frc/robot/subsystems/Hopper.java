// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.MotionMagicVoltage;


public class Hopper extends SubsystemBase {
  private final TalonFX leftHopperMotor;
  private final TalonFX rightHopperMotor;
  private final TalonFXConfiguration leftHopperConfig;
  private final TalonFXConfiguration rightHopperConfig;
  private double setPointOut = Constants.HopperPositions.EXTENDED_POSITION;
  private static final double POSITION_TOLERANCE = 0.2;
  private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0).withSlot(0);
  private final MotionMagicConfigs fastMMConfigs = new MotionMagicConfigs();
  private final MotionMagicConfigs slowMMConfigs = new MotionMagicConfigs();
  private boolean isFastMode = true;

  private final MotionMagicConfigs fastConfigs = new MotionMagicConfigs()
      .withMotionMagicCruiseVelocity(20.0)
      .withMotionMagicAcceleration(40.0)
      .withMotionMagicJerk(100.0);

  private final MotionMagicConfigs slowConfigs = new MotionMagicConfigs()
      .withMotionMagicCruiseVelocity(3.3)
      .withMotionMagicAcceleration(8.0)
      .withMotionMagicJerk(50.0);


  public Hopper() {
    leftHopperMotor = new TalonFX(Constants.Motors.LEFT_HOPPER, new CANBus("Indiana"));
    rightHopperMotor = new TalonFX(Constants.Motors.RIGHT_HOPPER, new CANBus("Indiana"));
    leftHopperConfig = new TalonFXConfiguration();
    rightHopperConfig = new TalonFXConfiguration();

    leftHopperConfig.Slot0.kS = 0.25;
    leftHopperConfig.Slot0.kV = 0.12;
    leftHopperConfig.Slot0.kA = 0.01;
    leftHopperConfig.Slot0.kP = 5.0;
    leftHopperConfig.Slot0.kI = 0.0;
    leftHopperConfig.Slot0.kD = 0.3;
    leftHopperConfig.CurrentLimits.SupplyCurrentLimit = 30.0;
    leftHopperConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    rightHopperConfig.Slot0.kS = 0.25;
    rightHopperConfig.Slot0.kV = 0.12;
    rightHopperConfig.Slot0.kA = 0.01;
    rightHopperConfig.Slot0.kP = 5.0;
    rightHopperConfig.Slot0.kI = 0.0;
    rightHopperConfig.Slot0.kD = 0.3;
    rightHopperConfig.CurrentLimits.SupplyCurrentLimit = 30.0;
    rightHopperConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    fastMMConfigs.MotionMagicCruiseVelocity = 20.0;
    fastMMConfigs.MotionMagicAcceleration = 40.0;
    fastMMConfigs.MotionMagicJerk = 100.0;

    slowMMConfigs.MotionMagicCruiseVelocity = 4.0;
    slowMMConfigs.MotionMagicAcceleration = 8.0;
    slowMMConfigs.MotionMagicJerk = 100.0;

    leftHopperConfig.MotionMagic = fastMMConfigs;
    rightHopperConfig.MotionMagic = fastMMConfigs;

    leftHopperMotor.getConfigurator().apply(leftHopperConfig);
    rightHopperMotor.getConfigurator().apply(rightHopperConfig);
  }

  public void setHopperMotorSpeed(double speed) {
    leftHopperMotor.set(speed);
    rightHopperMotor.set(speed);
  }

  public void stopHopperMotor() {
    leftHopperMotor.set(0);
    rightHopperMotor.set(0);
  }

  private void applyMMConfigs(MotionMagicConfigs configs) {
    if (isFastMode == (configs == fastMMConfigs)) return;
    isFastMode = (configs == fastMMConfigs);
    leftHopperMotor.getConfigurator().apply(configs);
    rightHopperMotor.getConfigurator().apply(configs);
  }

  public void setSetpoint(double setPoint) {
    applyMMConfigs(fastMMConfigs);
    leftHopperMotor.setControl(motionMagicRequest.withPosition(setPoint));
    rightHopperMotor.setControl(motionMagicRequest.withPosition(setPoint));
  }

  public void moveToOut() {
    applyMMConfigs(fastMMConfigs);
    leftHopperMotor.setControl(motionMagicRequest.withPosition(setPointOut));
    rightHopperMotor.setControl(motionMagicRequest.withPosition(setPointOut));
  }

  public void moveToIn() {
    applyMMConfigs(fastMMConfigs);
    leftHopperMotor.setControl(motionMagicRequest.withPosition(0.0));
    rightHopperMotor.setControl(motionMagicRequest.withPosition(0.0));
  }

  public void moveToOutSlow() {
    applyMMConfigs(slowMMConfigs);
    leftHopperMotor.setControl(motionMagicRequest.withPosition(setPointOut));
    rightHopperMotor.setControl(motionMagicRequest.withPosition(setPointOut));
  }

  public void moveToInSlow() {
    applyMMConfigs(slowMMConfigs);
    leftHopperMotor.setControl(motionMagicRequest.withPosition(0.0));
    rightHopperMotor.setControl(motionMagicRequest.withPosition(0.0));
  }

  public void stopHopper() {
    leftHopperMotor.setControl(motionMagicRequest.withPosition(leftHopperMotor.getPosition().getValueAsDouble()));
    rightHopperMotor.setControl(motionMagicRequest.withPosition(rightHopperMotor.getPosition().getValueAsDouble()));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Hopper Left Position", leftHopperMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Hopper Left Stator Current", leftHopperMotor.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Hopper Left Supply Current", leftHopperMotor.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Hopper Right Position", rightHopperMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Hopper Right Stator Current", rightHopperMotor.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Hopper Right Supply Current", rightHopperMotor.getSupplyCurrent().getValueAsDouble());
  }

}
