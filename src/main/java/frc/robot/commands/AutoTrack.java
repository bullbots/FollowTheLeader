// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Targeting;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;
import frc.robot.targeting.TargetingCalculator;
import frc.robot.targeting.TargetingCalculator.TargetingResult;

/**
 * Command that continuously tracks the hub/goal and adjusts turrets, hoods, and
 * shooter velocity.
 * Run this command while holding a button to enable auto-tracking.
 */
public class AutoTrack extends Command {

  private final Turret turret;
  private final Hood hood;
  private final Vision vision;
  private final Supplier<Pose2d> poseSupplier;

  private final Field2d m_field = new Field2d();

  private TargetingResult lastResult;

  /**
   * Creates a new AutoTrack command.
   *
   * @param turret       The turret subsystem
   * @param hood         The hood subsystem
   * @param shooter      The shooter subsystem
   * @param poseSupplier Supplier for the robot's current pose
   */
  public AutoTrack(Turret turret, Hood hood, Vision vision, Supplier<Pose2d> poseSupplier) {
    this.turret = turret;
    this.hood = hood;
    this.vision = vision;
    this.poseSupplier = poseSupplier;

    addRequirements(turret);

    SmartDashboard.putData("Field", m_field);
  }

  @Override
  public void initialize() {
    // Nothing special needed on init
  }

  @Override
  public void execute() {
    if (vision.hasSeenAprilTag()) {
      // Get current robot pose and calculate targeting
      Pose2d currentPose = poseSupplier.get();
      lastResult = TargetingCalculator.calculate(currentPose);

      double offset = SmartDashboard.getNumber("Turret/Offset", 0.0);
      turret.setSetpoint(lastResult.turretTarget.turretAngle + offset);

      if (lastResult.turretTarget.isValid) {

        hood.setSetpoints(lastResult.hoodAngleLeft, lastResult.hoodAngleRight);
      }

      // Update field visualization
      updateFieldVisualization(currentPose);
    }
    // Telemetry for debugging
    if (lastResult != null) {
      SmartDashboard.putNumber("AutoTrack/LeftTurretAngle", lastResult.turretTarget.turretAngle);
      SmartDashboard.putNumber("AutoTrack/RightTurretAngle", lastResult.turretTarget.turretAngle);
      SmartDashboard.putNumber("AutoTrack/Distance", lastResult.turretTarget.distanceMeters);
      SmartDashboard.putNumber("AutoTrack/HoodAngle", lastResult.hoodAngleLeft);
      SmartDashboard.putNumber("AutoTrack/LeftShooterVelocity", lastResult.leftShooterVelocity);
      SmartDashboard.putNumber("AutoTrack/RightShooterVelocity", lastResult.rightShooterVelocity);
      SmartDashboard.putBoolean("AutoTrack/LeftValid", lastResult.turretTarget.isValid);
      SmartDashboard.putBoolean("AutoTrack/RightValid", lastResult.turretTarget.isValid);
    }
  }

  private void updateFieldVisualization(Pose2d robotPose) {
    // Robot position
    m_field.setRobotPose(robotPose);

    // Hub marker — faces arbitrary direction, just marks the target location
    Translation2d hub = TargetingCalculator.getHubPosition();
    m_field.getObject("Hub").setPose(new Pose2d(hub, new Rotation2d()));

    // Active target position (hub or midfield fallback)
    m_field.getObject("Target").setPose(new Pose2d(lastResult.targetPosition, new Rotation2d()));

    // Left turret aim arrow — pose at the turret's field position, rotated toward
    // the hub
    Translation2d leftOffset = new Translation2d(Targeting.TURRET_OFFSET_X, Targeting.LEFT_TURRET_OFFSET_Y);
    Translation2d leftTurretPos = robotPose.getTranslation().plus(leftOffset.rotateBy(robotPose.getRotation()));
    Rotation2d leftAimAngle = Rotation2d.fromDegrees(lastResult.turretTarget.turretAngle)
        .plus(robotPose.getRotation());
    m_field.getObject("LeftTurretAim").setPose(new Pose2d(leftTurretPos, leftAimAngle));

    // Right turret aim arrow
    Translation2d rightOffset = new Translation2d(Targeting.TURRET_OFFSET_X, Targeting.RIGHT_TURRET_OFFSET_Y);
    Translation2d rightTurretPos = robotPose.getTranslation().plus(rightOffset.rotateBy(robotPose.getRotation()));
    Rotation2d rightAimAngle = Rotation2d.fromDegrees(lastResult.turretTarget.turretAngle)
        .plus(robotPose.getRotation());
    m_field.getObject("RightTurretAim").setPose(new Pose2d(rightTurretPos, rightAimAngle));
  }

  @Override
  public void end(boolean interrupted) {
    turret.holdCurrentPosition();
  }

  @Override
  public boolean isFinished() {
    // This command runs until interrupted (button released)
    return false;
  }

  /**
   * Returns whether both turrets are on target.
   *
   * @return true if both turrets are at their setpoints and targets are valid
   */
  public boolean isOnTarget() {
    return lastResult != null
        && lastResult.turretTarget.isValid
        && turret.atSetpoint()
        && hood.atSetpoint();
  }
}
