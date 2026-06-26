// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.targeting.TargetingCalculator;
import frc.robot.targeting.TargetingCalculator.TargetingResult;
import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */

public class ShootFuel extends Command {

  //private final Uptake uptake;
  private final Shooter shooter;
  private final SwerveSubsystem swerve;

  private final boolean dataCollectionMode = false;

  private final double leftVelocity;
  private final double rightVelocity;

  /** Creates a new ShootFuel. */
  public ShootFuel(Shooter shooter, SwerveSubsystem swerve, double leftVelocity, double rightVelocity) {
    //this.uptake = uptake;
    this.shooter = shooter;
    this.swerve = swerve;
    this.leftVelocity = leftVelocity;
    this.rightVelocity = rightVelocity;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //uptake.moveFuelUptake();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    TargetingResult result = TargetingCalculator.calculate(swerve.getPose());
    //System.out.println("Left Shooter Velocity: " + result.leftShooterVelocity);
    shooter.setKrakenMotorsSpeed(
        dataCollectionMode ? leftVelocity : result.leftShooterVelocity,
        dataCollectionMode ? rightVelocity : result.rightShooterVelocity);
  }

  // Called once the com*mand ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //uptake.stopUptakeMotor(); 
    shooter.stopShooterMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
