// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hopper;
import frc.robot.commands.HopperInSafety;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveHopperInFast extends Command {
  private final Hopper hopperSubsystem;
  private final HopperInSafety hopperInSafetyCommand;
  /** Creates a new ExtendHopper. */
  public MoveHopperInFast(Hopper hopperSubsystem, HopperInSafety hopperInSafetyCommand) {
    this.hopperSubsystem = hopperSubsystem;
    this.hopperInSafetyCommand = hopperInSafetyCommand;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(hopperSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(hopperInSafetyCommand.hopperSafeToPullin == true) {
      hopperSubsystem.moveToIn();
    }

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
