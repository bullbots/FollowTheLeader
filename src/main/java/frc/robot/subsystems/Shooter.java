// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Shooter extends SubsystemBase {
  private final TalonFX shooterLeft;
  private final TalonFX shooterRight;

  // Motion Magic control requests
  private final MotionMagicVelocityVoltage motionMagicLeft = new MotionMagicVelocityVoltage(0);
  private final MotionMagicVelocityVoltage motionMagicRight = new MotionMagicVelocityVoltage(0);

  public Shooter() {
    shooterLeft = new TalonFX(Constants.Motors.SHOOTER_LEFT);

    shooterRight = new TalonFX(Constants.Motors.SHOOTER_RIGHT);

    // Configure Kraken 1 for coast mode with Motion Magic
    TalonFXConfiguration kraken1Config = new TalonFXConfiguration();
    kraken1Config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    // Motion Magic configuration for smooth acceleration
    kraken1Config.MotionMagic.MotionMagicAcceleration = 400; // rotations/s^2
    kraken1Config.MotionMagic.MotionMagicJerk = 4000; // rotations/s^3

    // Velocity PID configuration
    kraken1Config.Slot0.kP = 0.1;
    kraken1Config.Slot0.kI = 0.0;
    kraken1Config.Slot0.kD = 0.0;
    kraken1Config.Slot0.kV = 0.12; // Feed-forward for velocity control

    // Current limiting configuration
    kraken1Config.CurrentLimits.SupplyCurrentLimit = 40; // Limit current to 40A
    kraken1Config.CurrentLimits.SupplyCurrentLowerLimit = 30; // Lower limit after sustained high current
    kraken1Config.CurrentLimits.SupplyCurrentLowerTime = 0.5; // Time before reducing to lower limit
    kraken1Config.CurrentLimits.SupplyCurrentLimitEnable = true; // Enable current limiting

    shooterLeft.getConfigurator().apply(kraken1Config);

    //////////////////

    // Configure Kraken 3 for coast mode with Motion Magic
    TalonFXConfiguration kraken3Config = new TalonFXConfiguration();
    kraken3Config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    // Motion Magic configuration for smooth acceleration
    kraken3Config.MotionMagic.MotionMagicAcceleration = 400; // rotations/s^2
    kraken3Config.MotionMagic.MotionMagicJerk = 4000; // rotations/s^3

    // Velocity PID configuration
    kraken3Config.Slot0.kP = 0.1;
    kraken3Config.Slot0.kI = 0.0;
    kraken3Config.Slot0.kD = 0.0;
    kraken3Config.Slot0.kV = 0.12; // Feed-forward for velocity control

    // Current limiting configuration
    kraken3Config.CurrentLimits.SupplyCurrentLimit = 40;
    kraken3Config.CurrentLimits.SupplyCurrentLowerLimit = 30;
    kraken3Config.CurrentLimits.SupplyCurrentLowerTime = 0.5;
    kraken3Config.CurrentLimits.SupplyCurrentLimitEnable = true;

    shooterRight.getConfigurator().apply(kraken3Config);

  }

  public void setKrakenMotorsSpeed(double speedLeft, double speedRight) {
    // Convert speed (-1 to 1) to velocity in rotations per second
    // Assuming max velocity of 100 RPS (adjust based on your shooter requirements)
    double maxVelocityRPS = 100.0;
    double leftVelocity = speedLeft * maxVelocityRPS;
    double rightVelocity = speedRight * maxVelocityRPS;

    // Use Motion Magic for smooth, controlled acceleration
    shooterLeft.setControl(motionMagicLeft.withVelocity(leftVelocity));
    shooterRight.setControl(motionMagicRight.withVelocity(rightVelocity));
  }

  /**
   * Stops both motors smoothly using Motion Magic (Kraken 2 and 4 will
   * automatically stop since they follow Kraken 1 and 3).
   * this should not need to be used :)
   */
  public void stopShooterMotors() {
    shooterLeft.setControl(new NeutralOut());
    shooterRight.setControl(new NeutralOut());
  }

  public void setShootersVelocity(double leftVelocity, double rightVelocity) {
    SmartDashboard.putNumber("Shooter Left Setpoint", leftVelocity);
    SmartDashboard.putNumber("Shooter Right Setpoint", rightVelocity);
    shooterLeft.set(leftVelocity);
    shooterRight.set(rightVelocity);
  }

  public void bothShootersShootHalf() {
    shooterLeft.set(0.6);
    shooterRight.set(0.6);
  }

  public double getLeftVelocity() {
    return shooterLeft.getVelocity().getValueAsDouble();
  }

  public double getRightVelocity() {
    return shooterRight.getVelocity().getValueAsDouble();
  }

  public double getLeftPosition() {
    return shooterLeft.getPosition().getValueAsDouble();
  }

  public double getRightPosition() {
    return shooterRight.getPosition().getValueAsDouble();
  }

  public void bothShootersShoot() {
    shooterLeft.set(1);
    shooterRight.set(1);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter Left Velocity (RPS)", getLeftVelocity());
    SmartDashboard.putNumber("Shooter Right Velocity (RPS)", getRightVelocity());
  }
}
