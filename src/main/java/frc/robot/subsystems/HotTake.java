// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class HotTake extends SubsystemBase {
  TalonFX hotTakeMotor;

  //TODO: if necessary, give this motion magic / magic motion control for smooth acceleration and deceleration, and current limiting for motor safety
  public HotTake() {
    hotTakeMotor = new TalonFX(Constants.Motors.HOT_TAKE, new CANBus("Indiana"));
  }

  public void setHotTakeSpeed(double speed) {
    hotTakeMotor.set(speed);
  }

  public void stopHotTakeMotor() {
    hotTakeMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
