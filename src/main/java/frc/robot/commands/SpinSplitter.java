// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Splitter;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SpinSplitter extends Command {
  private final Splitter m_splitter;
  private final double speed;
  /** Creates a new SpinSplitter. */
  public SpinSplitter(Splitter splitter, double speed) {
    m_splitter = splitter;
    this.speed = speed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_splitter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_splitter.setSplitterMotorSpeed(-0.2); // Set the splitter motor to spin at 50% speed
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_splitter.setSplitterMotorSpeed(speed); // Ensure the splitter motor continues to spin at the specified speed
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_splitter.stopSplitterMotor(); // Stop the splitter motor when the command ends
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
