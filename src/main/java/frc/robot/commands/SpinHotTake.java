// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HotTake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SpinHotTake extends Command {
  private final HotTake m_hotTake;
  /** Creates a new SpinHotTake. */
  public SpinHotTake(HotTake hotTake) {
    m_hotTake = hotTake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_hotTake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_hotTake.setHotTakeSpeed(-0.2); // Set the hot take motor to spin at 50% speed
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_hotTake.stopHotTakeMotor(); // Stop the hot take motor when the command ends
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
