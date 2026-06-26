// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.targeting.TargetingCalculator;
import frc.robot.targeting.TargetingCalculator.TargetingResult;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PassiveHood extends Command {
  Hood hood;
  private final boolean dataCollectionMode = false;
  private final double leftPosition;
  private final double rightPosition;
  private final SwerveSubsystem swerve;
  /** Creates a new PassiveHood. */
  public PassiveHood(Hood hood, SwerveSubsystem swerve, double leftPosition, double rightPosition) {
    this.hood = hood;
    this.leftPosition = leftPosition;
    this.rightPosition = rightPosition;
    this.swerve = swerve;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(hood);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    TargetingResult result = TargetingCalculator.calculate(swerve.getPose());
    hood.setSetpoints(
        dataCollectionMode ? leftPosition : result.hoodAngleLeft,
        dataCollectionMode ? rightPosition : result.hoodAngleRight);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
