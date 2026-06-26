// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hood;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveHood extends Command {
  private final Hood Hood;
  private final double pointLeft;
  private final double pointRight;
  /** Creates a new MoveHood. */
  public MoveHood(Hood hood, double setpointLeft, double setpointRight) {
    // Use addRequirements() here to declare subsystem dependencies.
    Hood = hood;
    pointLeft = setpointLeft;
    pointRight = setpointRight;
    addRequirements(hood);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Moving hood to setpoints: " + pointLeft + ", " + pointRight);
    Hood.setSetpoints(pointLeft, pointRight);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
