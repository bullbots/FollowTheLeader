// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveTurretsToZero extends Command {
  /** Creates a new MoveTurretsToZero. */
  private final Turret turret;
  private boolean isEnabled = false;

  public MoveTurretsToZero(Turret turret) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.turret = turret;

    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isEnabled = !isEnabled;
    SmartDashboard.putBoolean("isEnabled, turret zero", isEnabled);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(isEnabled == true){
    turret.setSetpoint(0);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
