package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Splitter;
import frc.robot.subsystems.HotTake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Burrito extends Command {
  /** Creates a new Shoot. */
  private final Splitter splitter;
  private final HotTake hotTake;
  
  long startTime = System.currentTimeMillis() +1000;

  public Burrito(Splitter splitter, HotTake hotTake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.splitter = splitter;
    this.hotTake = hotTake;

    addRequirements(splitter, hotTake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (startTime >= System.currentTimeMillis()){
      splitter.setSplitterMotorSpeed(0.5);
      hotTake.setHotTakeSpeed(0.5);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
   splitter.stopSplitterMotor();
   hotTake.stopHotTakeMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
