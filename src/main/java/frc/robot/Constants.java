// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final double MAX_SPEED = Units.feetToMeters(14.5);

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kCopilotControllerPort = 1;

    // Joystick Deadband
    public static final double DEADBAND = 0.15;
  }

  public static class DrivebaseConstants {
    // Time in seconds to wait before releasing motor brake after disable
    public static final double WHEEL_LOCK_TIME = 10.0;
  }
  
  public static class Motors {
    public static final int SHOOTER_LEFT = 9;
    public static final int SHOOTER_RIGHT = 11;
    public static final int HOOD_LEFT = 13;
    public static final int HOOD_RIGHT = 14;
    public static final int TURRETS = 15;
    public static final int HOT_TAKE = 16;
    public static final int SPLITTER_UPTAKE = 17; //this one
    public static final int LEFT_HOPPER = 18;
    public static final int RIGHT_HOPPER = 22;
    public static final int INTAKE = 19;
    public static final int ROLLERS = 20;
    //public static final int TURRET_ENCODER_LEFT = 22;
    //public static final int TURRET_ENCODER_RIGHT = 23;

  }

  public static class Offsets {
    public static final double PIVOT_ENCODER_OFFSET = 0.34033203125;
    public static final double RAW_PIVOT_DOWN_POS = -0.38989257812;
  }

  public static class HopperPositions {
    public static final double RETRACTED_POSITION = 0.0; // Fully retracted hopper position
    public static final double EXTENDED_POSITION = -26.199707; // Fully extended hopper position
  }

  public static class GearRatio {
    public static final double turret = 17.77;
    public static final double pivot = 222.22; // Set to actual gear ratio (motor rotations per pivot rotation)
  }

  public static class TurretConstants {
    // Motion Magic parameters (stored in degrees, converted to rotations when applied)
    public static final double kCruiseVelocity = 180.0;   // degrees per second
    public static final double kAcceleration = 360.0;     // degrees per second squared
    public static final double kJerk = 3600.0;            // degrees per second cubed

    // PID and feedforward gains (Phoenix 6 native units — rotations)
    public static final double kS = 0.5;            // static friction (volts)
    public static final double kV = 0.12;           // velocity feedforward (V per rotation/s)
    public static final double kA = 0.01;           // acceleration feedforward (V per rotation/s²)
    public static final double kP = 24.0;           // proportional gain
    public static final double kI = 0.0 / 360.0;   // integral gain (~0.002778)
    public static final double kD = 0;         // derivative gain

    // Soft limits (in degrees — converted to rotations when applied)
    public static final double kForwardSoftLimit = 90.0;   // +90 degrees
    public static final double kReverseSoftLimit = -90.0;  // -90 degrees

    // Current limits
    public static final double kSupplyCurrentLimit = 50.0; // Amps

    // Neutral mode
    public static final boolean kBrakeMode = true;
  }

  public static class Targeting {
    // Hub/goal positions in field coordinates (meters)
    // These are placeholder values - adjust based on your actual field/game
    public static final double BLUE_HUB_X = 4.6;  // Blue hub X position (meters)
    public static final double BLUE_HUB_Y = 4.0;  // Blue hub Y position (meters)
    public static final double RED_HUB_X = 11.938;  // Red hub X position (meters)
    public static final double RED_HUB_Y = 4.0;   // Red hub Y position (meters)

    // Turret physical offsets from robot center (meters)
    // Positive X is forward, positive Y is left
    public static final double TURRET_OFFSET_X = -0.1778;   // 7 inches in meters
    public static final double LEFT_TURRET_OFFSET_Y = 0.1588;    // 6.25 inches in meters (left of center, +Y in robot coords)
    public static final double RIGHT_TURRET_OFFSET_Y = -0.1588;  // 6.25 inches in meters (right of center, -Y in robot coords)

    // Hood angle range (in output rotations after gear ratio)
    public static final double HOOD_MIN_ANGLE = 0.0;
    public static final double HOOD_MAX_ANGLE = 0.7;

    // Distance thresholds for shooter/hood interpolation (meters)
    public static final double MIN_SHOOTING_DISTANCE = 1.0;
    public static final double MAX_SHOOTING_DISTANCE = 10.0;
  }
}

