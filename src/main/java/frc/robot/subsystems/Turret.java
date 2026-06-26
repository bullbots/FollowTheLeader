// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import frc.robot.trench.TrenchZones;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Turret extends SubsystemBase {

  private static final double kDegreesPerRotation = 360.0;

  private final TalonFX motor;
  private final TalonFXConfiguration motorConfig;
  private final MotionMagicVoltage m_mmRequest = new MotionMagicVoltage(0);
  private double setPoint;

  private final Supplier<Pose2d> poseSupplier;
  private boolean inTrench = false;
  private static final double POSITION_TOLERANCE = 3.6; // degrees (equivalent to 0.01 rotations)
  private static final double SETPOINT_DEADBAND = 0.75;          // degrees — base deadband
  private static final double SETPOINT_DEADBAND_VEL_SCALE = 0.02; // extra deadband per deg/s of turret velocity
  private int cycleCount = 0;

  // Mechanism2d for turret angle simulation
  private final Mechanism2d m_mech2d = new Mechanism2d(3, 3);
  private final MechanismRoot2d m_root = m_mech2d.getRoot("Turrets", 0.75, 1.5);
  private final MechanismLigament2d m_Actual =
      m_root.append(new MechanismLigament2d("Actual", 0.7, 0, 6, new Color8Bit(Color.kGreen)));
  private final MechanismLigament2d m_Setpoint =
      m_root.append(new MechanismLigament2d("Setpoint", 0.7, 0, 2, new Color8Bit(Color.kYellow)));

  public Turret(double setPoint, Supplier<Pose2d> poseSupplier) {
    this.poseSupplier = poseSupplier;
    this.setPoint = setPoint;

    motor = new TalonFX(Constants.Motors.TURRETS, new CANBus("Indiana"));

    motorConfig = new TalonFXConfiguration();

    // Gear ratio
    motorConfig.Feedback.SensorToMechanismRatio = Constants.GearRatio.turret;

    // PID + feedforward
    motorConfig.Slot0.kS = Constants.TurretConstants.kS;
    motorConfig.Slot0.kV = Constants.TurretConstants.kV;
    motorConfig.Slot0.kA = Constants.TurretConstants.kA;
    motorConfig.Slot0.kP = Constants.TurretConstants.kP;
    motorConfig.Slot0.kI = Constants.TurretConstants.kI;
    motorConfig.Slot0.kD = Constants.TurretConstants.kD;

    // MotionMagic (convert degrees to rotations)
    motorConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.TurretConstants.kCruiseVelocity / kDegreesPerRotation;
    motorConfig.MotionMagic.MotionMagicAcceleration = Constants.TurretConstants.kAcceleration / kDegreesPerRotation;
    motorConfig.MotionMagic.MotionMagicJerk = Constants.TurretConstants.kJerk / kDegreesPerRotation;

    motorConfig.MotorOutput.NeutralMode = Constants.TurretConstants.kBrakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast;

    // Soft limits (convert degrees to rotations)
    motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.TurretConstants.kForwardSoftLimit / kDegreesPerRotation;
    motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Constants.TurretConstants.kReverseSoftLimit / kDegreesPerRotation;

    // Current limits
    motorConfig.CurrentLimits.SupplyCurrentLimit = Constants.TurretConstants.kSupplyCurrentLimit;
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    // Apply configuration with retry
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = motor.getConfigurator().apply(motorConfig);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Turret: could not apply motor config. Error: " + status.toString());
    }

    // Hold position at startup
    this.setPoint = getPosition();

    SmartDashboard.putData("Turret Mechanism", m_mech2d);
    SmartDashboard.putNumber("Turret/Offset", 0.0);
  }

  @Override
  public void periodic() {
    // inTrench = TrenchZones.getInstance().isInsideAnyTrench(poseSupplier.get());
    // if (inTrench) {
    //   setPoint = 0;
    // }

    motor.setControl(m_mmRequest.withPosition(setPoint / kDegreesPerRotation).withSlot(0));

    m_Actual.setAngle(Rotation2d.fromDegrees(getPosition()));
    m_Setpoint.setAngle(Rotation2d.fromDegrees(setPoint));

    cycleCount++;
    if (cycleCount >= 10) {
      cycleCount = 0;
      SmartDashboard.putNumber("Turret Position", getPosition());
      SmartDashboard.putNumber("Turret Setpoint", setPoint);
      SmartDashboard.putBoolean("Turret In Trench", inTrench);
    }
  }

  public void setSetpoint(double position) {
    // if (inTrench) {
    //   return;
    // }
    double speed = Math.abs(motor.getVelocity().getValueAsDouble() * kDegreesPerRotation);
    double effectiveDeadband = SETPOINT_DEADBAND + SETPOINT_DEADBAND_VEL_SCALE * speed;
    if (Math.abs(position - setPoint) < effectiveDeadband) {
      return;
    }
    this.setPoint = position;
  }

  public boolean isInTrench() {
    return inTrench;
  }

  public double getSetpoint() {
    return setPoint;
  }

  public double getPosition() {
    return motor.getPosition().getValueAsDouble() * kDegreesPerRotation;
  }

  public boolean atSetpoint() {
    return Math.abs(getPosition() - setPoint) < POSITION_TOLERANCE;
  }

  public void holdCurrentPosition() {
    this.setPoint = getPosition();
  }

  public void stop() {
    motor.stopMotor();
  }
}
