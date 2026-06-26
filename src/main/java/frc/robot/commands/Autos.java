// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import frc.robot.targeting.InterpolationTables;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.commands.JustShoot;

public final class Autos {
  private static final SendableChooser<Command> commandChooser = new SendableChooser<>();

  /** Example static factory for an autonomous command. */
  public static Command exampleAuto(ExampleSubsystem subsystem) {
    return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
  }

  public static void load(Shooter shooter, Turret turret, Hood hood, SwerveSubsystem swerve) {
    commandChooser.setDefaultOption("do nothing", new PrintCommand("do nothing"));
    commandChooser.addOption("Just Shoot", new JustShoot(shooter, turret, hood, swerve));
    SmartDashboard.putData("Auto Chooser", commandChooser);
  }

  public static Command getSelected() {
    return commandChooser.getSelected();
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
