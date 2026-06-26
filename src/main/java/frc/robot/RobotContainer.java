// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoTrack;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.HailMary;
import frc.robot.commands.HopperInSafety;
import frc.robot.commands.JustShoot;
import frc.robot.commands.FrontIntake;
import frc.robot.commands.SetKrakens;
import frc.robot.commands.SetHoodToZero;
import frc.robot.commands.ShootFuel;
import frc.robot.commands.TurningTurrets;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Splitter;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Turret;
import frc.robot.targeting.InterpolationTables;
import frc.robot.targeting.TargetingCalculator;
import frc.robot.subsystems.Intake;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import swervelib.SwerveInputStream;
import frc.robot.trench.TrenchZones;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.File;
import frc.robot.subsystems.Hood;
import frc.robot.commands.MoveHood;
import frc.robot.subsystems.Hopper;
import frc.robot.commands.MoveHopperInFast;
import frc.robot.commands.MoveHopperInSlow;
import frc.robot.commands.PassiveHood;
import frc.robot.commands.Burrito;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.SpinHotTake;
import frc.robot.subsystems.HotTake;
import frc.robot.subsystems.Rollers;
import frc.robot.commands.SpinRollers;
import frc.robot.commands.SpinSplitter;
import frc.robot.commands.MoveHopperInFast;
import frc.robot.commands.MoveHopperInSlow;
import frc.robot.commands.MoveHopperOutFast;
import frc.robot.commands.MoveTurretsToZero;
import frc.robot.commands.LawnMower;
import frc.robot.commands.OhCrap;



/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final Hopper hopper = new Hopper();
  private final Shooter shooter = new Shooter();
  public static SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  private final SwerveSubsystem m_swerveSubsystem = drivebase;
  //private final Uptake uptake = new Uptake();
  private final HotTake m_HotTake = new HotTake();
  private final Rollers m_Rollers = new Rollers();
  private final Splitter splitter = new Splitter(.2);

  //private final Pivot pivot = new Pivot();
  private final Intake m_Intake = new Intake();
  //private final Hopper hopperSubsystem = new Hopper(Constants.HopperPositions.RETRACTED_POSITION);
  private final Turret turret = new Turret(0, () -> m_swerveSubsystem.getPose());
  private final Hood hood = new Hood(0, 0, () -> m_swerveSubsystem.getPose());
  //private final HailMary hailMary = new HailMary(shooter, hood, uptake, turret);


  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController xboxController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private final CommandJoystick guitarHero = new CommandJoystick(OperatorConstants.kCopilotControllerPort);

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(m_swerveSubsystem.getSwerveDrive(),
      () -> xboxController.getLeftY() * -1,
      () -> xboxController.getLeftX() * -1)
      .withControllerRotationAxis(() -> xboxController.getRightX() * -1)
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);

  // Target heading direction for trench warfare mode, captured when B is pressed
  private double trenchWarfareHeadingDirection = 1.0;

  /**
   * Drive stream for "trench warfare" mode - locks heading to be parallel to the player station
   * (uses captured heading direction), only allows forward/backward movement via left Y stick.
   */
  SwerveInputStream driveStraightAway = SwerveInputStream.of(m_swerveSubsystem.getSwerveDrive(),
      () -> xboxController.getLeftY() * -1,
      () -> 0.0)
      .withControllerHeadingAxis(() -> 0.0, () -> trenchWarfareHeadingDirection)
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);

  /**
   * Returns 1.0 if robot is closer to facing 0° (away from alliance wall),
   * or -1.0 if closer to facing 180° (toward alliance wall).
   * This snaps the robot to the nearest parallel-to-player-station heading.
   */
  private double getParallelHeadingDirection() {
    double headingDegrees = drivebase.getHeading().getDegrees();
    // Normalize to -180 to 180
    headingDegrees = ((headingDegrees % 360) + 360) % 360;
    if (headingDegrees > 180) {
      headingDegrees -= 360;
    }
    // If heading is between -90 and 90, we're closer to 0° (facing away)
    // Otherwise we're closer to 180° (facing toward)
    return (headingDegrees >= -90 && headingDegrees <= 90) ? 1.0 : -1.0;
  }

  /**
   * Returns true if the robot's heading is within tolerance of the target parallel heading.
   */
  private boolean isParallelToPlayerStation() {
    double headingDegrees = drivebase.getHeading().getDegrees();
    // Normalize to -180 to 180
    headingDegrees = ((headingDegrees % 360) + 360) % 360;
    if (headingDegrees > 180) {
      headingDegrees -= 360;
    }
    // Check if within 5 degrees of 0 or 180
    double targetDegrees = (trenchWarfareHeadingDirection > 0) ? 0 : 180;
    double error = Math.abs(headingDegrees - targetDegrees);
    if (error > 180) {
      error = 360 - error;
    }
    return error < 5.0;
  }

  /**
   * Keyboard input stream for simulation - uses raw axis for rotation.
   */
  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(m_swerveSubsystem.getSwerveDrive(),
      () -> -xboxController.getLeftY(),
      () -> -xboxController.getLeftX())
      .withControllerRotationAxis(() -> xboxController.getRawAxis(2))
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);

  /**
   * Keyboard input stream with direct angle control for simulation.
   */
  SwerveInputStream driveDirectAngleKeyboard = driveAngularVelocityKeyboard.copy()
      .withControllerHeadingAxis(
          () -> Math.sin(xboxController.getRawAxis(2) * Math.PI) * (Math.PI * 2),
          () -> Math.cos(xboxController.getRawAxis(2) * Math.PI) * (Math.PI * 2))
      .headingWhile(true);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Initialize trench zones eagerly to avoid parsing delays during competition
    TrenchZones.initialize();

    // Initialize interpolation tables for targeting calculator
    TargetingCalculator.setInterpolationTables(new InterpolationTables());

    // Load autonomous chooser with subsystems
    //Autos.load(uptake, shooter, turret, hood, m_swerveSubsystem);

    // Configure the trigger bindings
    configureBindings();

    hood.setDefaultCommand(new PassiveHood(hood, m_swerveSubsystem, 4, 4));
    turret.setDefaultCommand(
      new AutoTrack(turret, hood, m_swerveSubsystem.getVision(), () -> m_swerveSubsystem.getPose())
  );

    DriverStation.silenceJoystickConnectionWarning(true);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Create drive commands
    Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveFieldOrientedDirectAngleKeyboard = drivebase.driveFieldOriented(driveDirectAngleKeyboard);

    // Set default drive command based on simulation or real robot
    if (RobotBase.isSimulation()) {
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
    } else {
      drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);
    }

    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Hold B button for "trench warfare" mode:
    // 1. Capture the nearest parallel heading (0° or 180°)
    // 2. Rotate in place until parallel to player station
    // 3. Then allow forward/backward movement while maintaining heading
    // xboxController.b().whileTrue(Commands.sequence(
    //     // Capture the target heading direction when B is first pressed
    //     Commands.runOnce(() -> trenchWarfareHeadingDirection = getParallelHeadingDirection()),
    //     // Rotate in place until parallel (no translation, just heading control)
    //     m_swerveSubsystem.driveCommand(() -> 0.0, () -> 0.0,
    //         () -> 0.0, () -> trenchWarfareHeadingDirection)
    //         .until(this::isParallelToPlayerStation),
    //     // Now allow forward/backward movement while locked to parallel heading
    //     Commands.parallel(
    //         m_swerveSubsystem.driveFieldOriented(driveStraightAway),
    //         Commands.print("TODO: Zero hood when subsystem is added"),
    //         Commands.print("TODO: Zero turrets when subsystem is added")
    //     )
    // ));

    // SetKrakens test (hold left trigger)
    //xboxController.leftTrigger().whileTrue(new SetKrakens(shooter, 1, 1));

    // Simulation-specific bindings
    if (RobotBase.isSimulation()) {
      // Start button resets odometry to center of field
      xboxController.start().onTrue(
          Commands.runOnce(() -> m_swerveSubsystem.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
    }
    //m_driverController.b().whileTrue(new SetKrakens(m_shooter, 1, 1));

    // Alex's controls
    // Press start button to zero the gyro (reset field-oriented forward direction)
    xboxController.start().onTrue(m_swerveSubsystem.runOnce(() -> m_swerveSubsystem.zeroGyro()));

    // Press X button to lock swerve wheels in X pattern (defense mode)
    //xboxController.x().whileTrue(m_swerveSubsystem.run(() -> m_swerveSubsystem.lock()));

    //Press right Trigger to shoot the balls
    //xboxController.rightTrigger().whileTrue(new UptakeFuel(uptake));
    //xboxController.rightTrigger().whileTrue(new ShootFuel(shooter, m_swerveSubsystem, 5, 5)); //todo: adjust velocity values as needed
    //Press y to spit out the balls from the intake
    //xboxController.y().whileTrue(new Burrito(uptake));

    //xboxController.a().onTrue(new SpinSplitter(splitter, 0.2));

    xboxController.rightTrigger().whileTrue(new ParallelCommandGroup( // Start shooters, then start loading fuel once ready. May need tuning, may not.
      new ShootFuel(shooter, m_swerveSubsystem, 10,10),
      new SequentialCommandGroup(
        new WaitCommand(1),
        new SpinSplitter(splitter, -0.5)
      ),
      new SequentialCommandGroup(
        new WaitCommand(1.25),
        new ParallelCommandGroup(
          new SpinRollers(m_Rollers),
          new SpinHotTake(m_HotTake)
        )
      )
    ));

    xboxController.a().whileTrue(new MoveHopperOutFast(hopper));

    xboxController.b().whileTrue(new LawnMower(m_Intake));

    xboxController.x().whileTrue(m_swerveSubsystem.run(() -> m_swerveSubsystem.lock()));

    // Start button: reset gyro (keep current position, zero rotation)
    xboxController.start().onTrue(
      Commands.runOnce(() -> m_swerveSubsystem.resetOdometry(
          new Pose2d(m_swerveSubsystem.getPose().getTranslation(), Rotation2d.fromDegrees(0))
        ))
    );

    xboxController.back().onTrue(new OhCrap(hopper));

    xboxController.leftTrigger().whileTrue(new FrontIntake(m_Intake));

    HopperInSafety hopperInSafety = new HopperInSafety();
    xboxController.leftBumper().whileTrue(hopperInSafety);

    xboxController.rightBumper().whileTrue(new MoveHopperInFast(hopper, hopperInSafety));
    // xboxController.leftBumper().onTrue(new MoveHopperOutFast(hopper, Constants.HopperPositions.EXTENDED_POSITION));
    // xboxController.rightBumper().onTrue(new MoveHopperInFast(hopper, Constants.HopperPositions.RETRACTED_POSITION));
    // xboxController.povUp().whileTrue(new SpinHotTake(m_HotTake)); // run hot take for 0.5 seconds when trigger is released


    guitarHero.povUp().whileTrue(new MoveHopperInSlow(hopper)); //Placeholder for moving hopper in slowly (co-driver). TODO: Add functionality to make hopper move slowly.
    guitarHero.button(4).onTrue(new OhCrap(hopper));
    guitarHero.button(3).onTrue(new MoveTurretsToZero(turret));
    guitarHero.button(2).whileTrue(new SetHoodToZero(hood));
    //xboxController.povLeft().whileTrue(new SpinRollers(m_Rollers)); // run rollers for 0.5 seconds when trigger is released
    //m_driverController.leftBumper().whileTrue(new TurningTurrets(turret, 2.5, -2.5));
    //m_driverController.rightBumper().whileTrue(new TurningTurrets(turret, -2.5, 2.5));

    // xboxController.leftBumper().onTrue(new ParallelCommandGroup(
    //   new Fold(pivot),
    //   new SequentialCommandGroup(
    //     new WaitCommand(5.0),
    //     new MoveHopper(hopperSubsystem, Constants.HopperPositions.RETRACTED_POSITION)
    //   )
    // )); // fold pivot, wait 5 seconds, then retract hopper. make this shorter if the neo moves faster.

    // xboxController.rightBumper().onTrue(new ParallelCommandGroup(
    //   new MoveHopper(hopperSubsystem, Constants.HopperPositions.EXTENDED_POSITION),
    //   new SequentialCommandGroup(
    //     new WaitCommand(3.0)
    //     //new UnFold(pivot)
    //   )
    // )); // extend hopper, wait 3 seconds, then unfold pivot

    //xboxController.povRight().onTrue(new UnFold(pivot));

    // Hold POV left for manual trench mode:
    // Driver manually rotates robot first, then holds to snap to nearest 0/180 and lock forward/backward movement
    // xboxController.povLeft().whileTrue(Commands.sequence(
    //     Commands.runOnce(() -> trenchWarfareHeadingDirection = getParallelHeadingDirection()),
    //     m_swerveSubsystem.driveFieldOriented(driveStraightAway)
    // ));

    //Press down on the d-pad to test moving the hood down (temporary)
    //guitarHero.povDown().onTrue(new MoveHood(hood, 0, 0));

    //Press up on the d-pad to test moving the hood up (temporary)
    //guitarHero.povUp().onTrue(new MoveHood(hood, 0.68, -0.68));

    //Elijah's Controls
    //guitarHero.button(1).whileTrue(new HailMary(shooter, hood, uptake, turret));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return Autos.getSelected();
  }

  /**
   * Sets the motor brake mode.
   *
   * @param brake True to set motors to brake mode, false for coast.
   */
  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }
}
