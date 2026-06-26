// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.targeting;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.Targeting;

/**
 * Calculates turret angles, hood angles, and shooter velocities for auto-targeting.
 *
 * Coordinate system:
 * - Field coordinates: +X toward red alliance wall, +Y toward left side when viewing from blue
 * - Robot coordinates: +X forward, +Y left
 */
public class TargetingCalculator {
    private static InterpolationTables interpolationTables;
    private static boolean spoofInitialized = false;

  public static void setInterpolationTables(InterpolationTables tables) {
    interpolationTables = tables;
    SmartDashboard.putBoolean("Distance Spoof Enabled", false);
    SmartDashboard.putNumber("Distance Spoof (meters)", 2.0);
  }
  /**
   * Result of targeting calculations for one turret.
   */
  public static class TurretTarget {
    public final double turretAngle;      // Turret angle in rotations (output shaft)
    public final double distanceMeters;   // Distance to target in meters
    public final boolean isValid;         // Whether the target is achievable

    public TurretTarget(double turretAngle, double distanceMeters, boolean isValid) {
      this.turretAngle = turretAngle;
      this.distanceMeters = distanceMeters;
      this.isValid = isValid;
    }
  }

  /**
   * Result of targeting calculations for both turrets and hoods.
   */
  public static class TargetingResult {
    public final TurretTarget turretTarget;
    public final double hoodAngleLeft;          // Hood angle in rotations
    public final double hoodAngleRight;         // Hood angle in rotations
    public final double leftShooterVelocity;    // Left shooter velocity as fraction (0-1)
    public final double rightShooterVelocity;   // Right shooter velocity as fraction (0-1)
    public final Translation2d targetPosition;  // Field-relative target being aimed at

    public TargetingResult(TurretTarget turretTarget,
                           double hoodAngleLeft, double hoodAngleRight,
                           double leftShooterVelocity, double rightShooterVelocity,
                           Translation2d targetPosition) {
      this.turretTarget = turretTarget;
      this.hoodAngleLeft = hoodAngleLeft;
      this.hoodAngleRight = hoodAngleRight;
      this.leftShooterVelocity = leftShooterVelocity;
      this.rightShooterVelocity = rightShooterVelocity;
      this.targetPosition = targetPosition;
    }
  }
  
    /**
     * Gets the hub position based on alliance color.
     *
     * @return Hub position in field coordinates
     */
    public static Translation2d getHubPosition() {
      if (RobotBase.isSimulation()) {
        return new Translation2d(4.75, 4);
      }
      var alliance = DriverStation.getAlliance();
      if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
        return new Translation2d(Targeting.RED_HUB_X, Targeting.RED_HUB_Y);
      }
      return new Translation2d(Targeting.BLUE_HUB_X, Targeting.BLUE_HUB_Y);
    }
  
    //this method should be called when the robot is in the middle of the field m the hub
  public static Translation2d getMidfieldTargetPosition(Pose2d robotPose) {
  
      if (RobotBase.isSimulation()) {
        if (robotPose.getY() > 4) {
          return new Translation2d(2, 6);
        } else {
          return new Translation2d(2, 2);
        }
      }
  
      var alliance = DriverStation.getAlliance();
      if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
        if (robotPose.getY() > Targeting.RED_HUB_Y) {
          return new Translation2d(Targeting.RED_HUB_X + 3, Targeting.RED_HUB_Y + 3);
        } else {
          return new Translation2d(Targeting.RED_HUB_X + 3, Targeting.RED_HUB_Y - 3);
        }
      } else {
        if (robotPose.getY() > Targeting.BLUE_HUB_Y) {
          return new Translation2d(Targeting.BLUE_HUB_X - 3, Targeting.BLUE_HUB_Y + 3);
        } else {
          return new Translation2d(Targeting.BLUE_HUB_X - 3, Targeting.BLUE_HUB_Y - 3);
        }
      }
    }
  
    /**
     * Calculates targeting data for both turrets.
     * 
     * @param robotPose Current robot pose in field coordinates
     * @return TargetingResult with angles and velocities for all mechanisms
     */
    public static TargetingResult calculate(Pose2d robotPose) {
      if (!spoofInitialized) {
        SmartDashboard.putBoolean("Distance Spoof Enabled", false);
        SmartDashboard.putNumber("Distance Spoof (meters)", 1.269812025);
        spoofInitialized = true;
      }
      Translation2d hubPosition = getHubPosition();

      // If the robot has crossed past the hub (too close), aim at a midfield target instead.
      // Blue alliance: robot is past the hub when X < hub X (robot is too close to hub)
      // Red alliance: robot is past the hub when X > hub X (robot is too close to hub)
      Translation2d targetPosition;
      var alliance = DriverStation.getAlliance();
      if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Blue) {
        if (robotPose.getX() < hubPosition.getX()) {
          targetPosition = getMidfieldTargetPosition(robotPose);
        } else {
          targetPosition = hubPosition;
        }
      } else {
        if (robotPose.getX() < hubPosition.getX()) {
          targetPosition = getMidfieldTargetPosition(robotPose);
        } else {
          targetPosition = hubPosition;
        }
      }

      TurretTarget turretTarget = calculateTurretTarget(
          robotPose,
          targetPosition,
          new Translation2d(Targeting.TURRET_OFFSET_X, 0)
      );

      // Calculate for left turret
      double leftDistance = calculateTurretDistanceFromTarget(
          robotPose,
          targetPosition,
          new Translation2d(Targeting.TURRET_OFFSET_X, Targeting.LEFT_TURRET_OFFSET_Y)
      );

      // Calculate for right turret
      double rightDistance = calculateTurretDistanceFromTarget(
          robotPose,
          targetPosition,
          new Translation2d(Targeting.TURRET_OFFSET_X, Targeting.RIGHT_TURRET_OFFSET_Y)
      );
  
      if (SmartDashboard.getBoolean("Distance Spoof Enabled", false)) {
        leftDistance = SmartDashboard.getNumber("Distance Spoof (meters)", 1.269812025);
        rightDistance = leftDistance;
      }

      // Calculate hood angles based on distance using separate left/right tables
      double hoodAngleLeft = calculateLeftHoodAngle(leftDistance);
      double hoodAngleRight = calculateRightHoodAngle(rightDistance);

      // Calculate shooter velocities based on distance using separate left/right tables
      double leftShooterVelocity = calculateLeftShooterVelocity(leftDistance);
      double rightShooterVelocity = calculateRightShooterVelocity(rightDistance);

      return new TargetingResult(turretTarget, hoodAngleLeft, hoodAngleRight, leftShooterVelocity, rightShooterVelocity, targetPosition);
    }
  
    /**
     * Calculates the turret angle for a single turret.
     *
     * @param robotPose Robot pose in field coordinates
     * @param targetPosition Target position in field coordinates
     * @param turretOffset Turret offset from robot center in robot coordinates
     * @return TurretTarget with angle and distance
     */
    private static TurretTarget calculateTurretTarget(Pose2d robotPose, Translation2d targetPosition, Translation2d turretOffset) {
      // Step 1: Transform turret offset from robot coords to field coords
      Translation2d turretOffsetField = turretOffset.rotateBy(robotPose.getRotation());

      // Step 2: Get turret position in field coords
      Translation2d turretFieldPosition = robotPose.getTranslation().plus(turretOffsetField);

      // Step 3: Calculate vector from turret to target in field coords
      Translation2d vectorToTarget = targetPosition.minus(turretFieldPosition);
  
      // Step 4: Transform vector to robot coordinates
      // Rotate by negative robot heading to convert field -> robot coords
      Translation2d vectorToTargetRobot = vectorToTarget.rotateBy(robotPose.getRotation().unaryMinus());
  
      // Step 5: Calculate turret angle using atan2
      // atan2(y, x) gives angle from +X axis (robot forward)
      // Shift by -π so that 0 = robot backward (turret zero position)
      double angleRadians = Math.atan2(vectorToTargetRobot.getY(), vectorToTargetRobot.getX()) - Math.PI;
      if (angleRadians <= -Math.PI) angleRadians += 2.0 * Math.PI;

      // Step 6: Convert radians to turret rotations
      // One full rotation = 2*PI radians = 1.0 turret rotation
      double turretAngleDeg = Math.toDegrees(angleRadians);

      SmartDashboard.putNumber("Turret Angle (deg)", turretAngleDeg);

      // Step 7: Calculate distance
      double distanceMeters = vectorToTarget.getNorm();

      //Step 8: set a private variable to distanceMeters``
      //distanceFromTarget = distanceMeters;

      boolean isValid = turretAngleDeg >= Constants.TurretConstants.kReverseSoftLimit
                     && turretAngleDeg <= Constants.TurretConstants.kForwardSoftLimit
                     && distanceMeters >= Targeting.MIN_SHOOTING_DISTANCE
                     && distanceMeters <= Targeting.MAX_SHOOTING_DISTANCE;

      return new TurretTarget(turretAngleDeg, distanceMeters, isValid);
    }

    private static double calculateTurretDistanceFromTarget(Pose2d robotPose, Translation2d targetPosition, Translation2d turretOffset) {
      // Step 1: Transform turret offset from robot coords to field coords
      Translation2d turretOffsetField = turretOffset.rotateBy(robotPose.getRotation());

      // Step 2: Get turret position in field coords
      Translation2d turretFieldPosition = robotPose.getTranslation().plus(turretOffsetField);

      // Step 3: Calculate vector from turret to target in field coords
      Translation2d vectorToTarget = targetPosition.minus(turretFieldPosition);
  
      // Step 4: Calculate distance
      double distanceMeters = vectorToTarget.getNorm();
  
      return distanceMeters;
    }
  /**
   * Calculates hood angle based on distance to target.
   * Uses linear interpolation between min and max hood angles.
   *
   * @param distanceMeters Distance to target in meters
   * @return Hood angle in rotations (for left hood; negate for right)
   */
  private static double calculateLeftHoodAngle(double distanceMeters) {
    double clampedDistance = Math.max(Targeting.MIN_SHOOTING_DISTANCE,
                                      Math.min(distanceMeters, Targeting.MAX_SHOOTING_DISTANCE));
    return interpolationTables.getLeftInterpolatedHoodAngle(clampedDistance);
  }

  private static double calculateRightHoodAngle(double distanceMeters) {
    double clampedDistance = Math.max(Targeting.MIN_SHOOTING_DISTANCE,
                                      Math.min(distanceMeters, Targeting.MAX_SHOOTING_DISTANCE));
    return interpolationTables.getRightInterpolatedHoodAngle(clampedDistance);
  }

  /**
   * Calculates shooter velocity based on distance to target.
   * Uses linear interpolation - farther targets need more velocity.
   *
   * @param distanceMeters Distance to target in meters
   * @return Shooter velocity as fraction (0.0 to 1.0)
   */
  private static double calculateLeftShooterVelocity(double distanceMeters) {
    double clampedDistance = Math.max(Targeting.MIN_SHOOTING_DISTANCE,
                                      Math.min(distanceMeters, Targeting.MAX_SHOOTING_DISTANCE));
    return interpolationTables.getLeftInterpolatedShooterSpeed(clampedDistance);
  }

  private static double calculateRightShooterVelocity(double distanceMeters) {
    double clampedDistance = Math.max(Targeting.MIN_SHOOTING_DISTANCE,
                                      Math.min(distanceMeters, Targeting.MAX_SHOOTING_DISTANCE));
    return interpolationTables.getRightInterpolatedShooterSpeed(clampedDistance);
  }
}
