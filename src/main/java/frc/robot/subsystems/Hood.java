// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.Motors;
import frc.robot.trench.TrenchZones;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;

public class Hood extends SubsystemBase {

  private SparkMax motorLeft;
  private SparkMax motorRight;
  private SparkMaxConfig motorConfigLeft;
  private SparkMaxConfig motorConfigRight;
  private SparkClosedLoopController closedLoopControllerLeft;
  private SparkClosedLoopController closedLoopControllerRight;
  private RelativeEncoder encoderLeft;
  private RelativeEncoder encoderRight;
  private double setPointLeft;
  private double setPointRight;
  private final Supplier<Pose2d> poseSupplier;
  private boolean inTrench = false;
  private static final double POSITION_TOLERANCE = 0.5;
  private int cycleCount = 0;

  public Hood(double setPointLeft, double setPointRight, Supplier<Pose2d> poseSupplier) {
    this.poseSupplier = poseSupplier;
    this.setPointLeft = setPointLeft;
    this.setPointRight = setPointRight;

    /*
     * Initialize the SPARK MAX motors and get their encoder and closed loop
     * controller objects for later use.
     */
    motorLeft = new SparkMax(Motors.HOOD_LEFT, MotorType.kBrushless);
    closedLoopControllerLeft = motorLeft.getClosedLoopController();
    encoderLeft = motorLeft.getEncoder();

    motorRight = new SparkMax(Motors.HOOD_RIGHT, MotorType.kBrushless);
    closedLoopControllerRight = motorRight.getClosedLoopController();
    encoderRight = motorRight.getEncoder();

    /*
     * Create new SPARK MAX configuration objects. These will store the
     * configuration parameters for the SPARK MAX motors.
     */
    motorConfigLeft = new SparkMaxConfig();
    motorConfigRight = new SparkMaxConfig();

    /*
     * Configure the encoder conversion factors if needed.
     */
    motorConfigLeft.encoder
        .positionConversionFactor(1.0 / 20.0)
        .velocityConversionFactor(1.0 / 20.0);

    motorConfigRight.encoder
        .positionConversionFactor(1.0 / 20.0)
        .velocityConversionFactor(1.0 / 20.0);

    /*
     * Configure the closed loop controller with feedback sensor and PID values.
     */
    motorConfigLeft.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(3.0)
        .i(0)
        .d(0)
        .outputRange(-1, 1);

    motorConfigRight.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(3.0)
        .i(0)
        .d(0)
        .outputRange(-1, 1);

    /*
     * Configure MAXMotion parameters for position control.
     */
    motorConfigLeft.closedLoop.maxMotion
        .cruiseVelocity(80)
        .maxAcceleration(80)
        .allowedProfileError(0.05);

    motorConfigRight.closedLoop.maxMotion
        .cruiseVelocity(80)
        .maxAcceleration(80)
        .allowedProfileError(0.05);

    /*
     * Configure current limits.
     */
    motorConfigLeft.smartCurrentLimit(40, 30, 1000);
    motorConfigRight.smartCurrentLimit(40, 30, 1000);
    motorConfigRight.inverted(true);
    motorConfigLeft.idleMode(IdleMode.kCoast);
    motorConfigRight.idleMode(IdleMode.kCoast);

    /*
     * Apply the configuration to the SPARK MAX motors.
     *
     * kResetSafeParameters is used to get the SPARK MAX to a known state.
     * kNoPersistParameters means configuration won't be saved across power cycles.
     */
    motorLeft.configure(motorConfigLeft, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    motorRight.configure(motorConfigRight, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void periodic() {
    // Update trench state and lock setpoints to 0 when inside
    inTrench = TrenchZones.getInstance().isInsideAnyTrench(poseSupplier.get());
    if (inTrench) {
      setPointLeft = 0;
      setPointRight = 0;
    }

    /*
     * Set the setpoint for the closed loop controller with MAXMotionPositionControl
     * as the control type.
     */
    closedLoopControllerLeft.setSetpoint(setPointLeft, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
    closedLoopControllerRight.setSetpoint(setPointRight, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);

    // Display encoder positions and trench status (throttled to every 10 cycles)
    cycleCount++;
    if (cycleCount >= 10) {
      cycleCount = 0;
      SmartDashboard.putNumber("Hood Left Position", encoderLeft.getPosition());
      SmartDashboard.putNumber("Hood Right Position", encoderRight.getPosition());
      SmartDashboard.putNumber("Hood Left Setpoint", setPointLeft);
      SmartDashboard.putNumber("Hood Right Setpoint", setPointRight);
      SmartDashboard.putBoolean("Hood In Trench", inTrench);
    }
  }

  private static final double SETPOINT_CHANGE_THRESHOLD = 0.01;

  public void setSetpoints(double left, double right) {
    if (inTrench) {
      //return; // Ignore setpoint changes while in trench
    }
    if (Math.abs(left - setPointLeft) > SETPOINT_CHANGE_THRESHOLD) {
      this.setPointLeft = left;
    }
    if (Math.abs(right - setPointRight) > SETPOINT_CHANGE_THRESHOLD) {
      this.setPointRight = right;
    }
  }

  public boolean isInTrench() {
    return inTrench;
  }

  public double getSetpointLeft() {
    return setPointLeft;
  }

  public double getSetpointRight() {
    return setPointRight;
  }

  public double getPositionLeft() {
    return encoderLeft.getPosition();
  }

  public double getPositionRight() {
    return encoderRight.getPosition();
  }

  public boolean atSetpoint() {
    return Math.abs(encoderLeft.getPosition() - setPointLeft) < POSITION_TOLERANCE
        && Math.abs(encoderRight.getPosition() - setPointRight) < POSITION_TOLERANCE;
  }

  public void bothHoodsUp(){
    setPointLeft = 0.663;
    setPointRight = -0.663;
  }


  public void stop() {
    motorLeft.stopMotor();
    motorRight.stopMotor();
  }
}
