// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class HailMary extends Command {

  private final Shooter shooter;
  private final Turret turret;
  private final Hood hood;

  /** Creates a new HailMary. */
  public HailMary(Shooter shooter, Hood hood, Turret turret) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    this.hood = hood;
    this.turret = turret;
    addRequirements(shooter, hood, turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turret.setSetpoint(0);
    hood.bothHoodsUp();
    shooter.bothShootersShoot();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopShooterMotors();
    // uptake.stopUptakeMotor(); // Uncomment if Uptake subsystem is used
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
