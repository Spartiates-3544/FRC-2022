// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.DeployClimberCommand;
import frc.robot.commands.DriveStraightCommand;
import frc.robot.commands.DriveStraightReverseCommand;
import frc.robot.commands.FeedBallsShooterCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.RetractClimberCommand;
import frc.robot.commands.ReverseIntakeCommand;
import frc.robot.commands.RunConveyorCommand;
import frc.robot.commands.RunConveyorReverseCommand;
import frc.robot.commands.RunIndexorColorCommand;
import frc.robot.commands.RunIndexorCommand;
import frc.robot.commands.RunIndexorProximityCommand;
import frc.robot.commands.SpinIntakeCommand;
import frc.robot.commands.SpinUpShooterCommand;
import frc.robot.commands.SpinUpShooterDistanceCommand;
import frc.robot.commands.StartIntakeCommand;
import frc.robot.commands.StopIntakeCommand;
import frc.robot.commands.StopSpinningIntakeCommand;
import frc.robot.commands.TrackTargetCommand;
import frc.robot.commands.TurnAngleCommand;
import frc.robot.commands.TurnAnglePIDCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IndexorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DrivetrainSubsystem m_drivetrain = new DrivetrainSubsystem();
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();
  private final IndexorSubsystem m_indexor = new IndexorSubsystem();
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final TurretSubsystem m_turret = new TurretSubsystem();
  private final LimelightSubsystem m_limelight = new LimelightSubsystem();
  private final XboxController m_controller = new XboxController(0);
  private final Joystick m_joystick = new Joystick(1);
  private final ClimberSubsystem m_climber = new ClimberSubsystem();
  private final ConveyorSubsystem m_conveyor = new ConveyorSubsystem();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    m_drivetrain.setDefaultCommand(new DefaultDriveCommand(m_drivetrain, m_controller));
   //m_turret.setDefaultCommand(new RunCommand(() -> m_turret.setTurret(m_controller.getRightX()), m_turret));
    //m_intake.setDefaultCommand(new IntakeCommand(m_intake));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    //Track target enable
    new JoystickButton(m_joystick, 2).toggleWhenPressed(new TrackTargetCommand(m_turret), true);
    
    //Pickup
    new JoystickButton(m_joystick, 3).toggleWhenPressed(new ParallelCommandGroup(new IntakeCommand(m_intake), new RunConveyorCommand(m_conveyor), new RunIndexorColorCommand(m_indexor)));

    //Reverse intake/conveyor
    new JoystickButton(m_joystick, 6).toggleWhenPressed(new ParallelCommandGroup(new ReverseIntakeCommand(m_intake), new RunConveyorReverseCommand(m_conveyor)));

    //Climber
    new JoystickButton(m_controller, Button.kLeftBumper.value).whenPressed(new DeployClimberCommand(m_climber));
    new JoystickButton(m_controller, Button.kRightBumper.value).whenPressed(new RetractClimberCommand(m_climber));

    //Shoot
    new JoystickButton(m_joystick, 1).toggleWhenPressed(new ParallelCommandGroup(new SpinUpShooterCommand(30000, m_shooter), new TrackTargetCommand(m_turret), new SequentialCommandGroup(new WaitCommand(4), new ParallelCommandGroup(new RunIndexorCommand(m_indexor), new RunConveyorCommand(m_conveyor)))));

    //Run Indexor(1sec)
    new JoystickButton(m_joystick, 5).whenPressed(new RunIndexorColorCommand(m_indexor).withTimeout(0.25));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

   
  public Command getAutonomousCommand() {
        
    /*
    // Create a voltage constraint to ensure we don't accelerate too fast
        var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
                Constants.DriveConstants.ksVolts,
                Constants.DriveConstants.kvVoltSecondsPerMeter,
                Constants.DriveConstants.kaVoltSecondsSquaredPerMeter),
            Constants.DriveConstants.kDriveKinematics,

            10);
          
      TrajectoryConfig config =
        new TrajectoryConfig(Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                             Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(Constants.DriveConstants.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);
    */

    /*
      Trajectory trajectory = PathPlanner.loadPath(Constants.AutoConstants.trajectoryName, Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared);

      RamseteCommand ramseteCommand =
      new RamseteCommand(
          trajectory,
          m_drivetrain::getPose,
          new RamseteController(Constants.AutoConstants.kRamseteB, Constants.AutoConstants.kRamseteZeta),
          new SimpleMotorFeedforward(
              Constants.DriveConstants.kS,
              Constants.DriveConstants.kV,
              Constants.DriveConstants.kA),
          Constants.DriveConstants.kDriveKinematics,
          m_drivetrain::getWheelSpeeds,
          new PIDController(Constants.DriveConstants.kPDriveVel, 0, 0),
          new PIDController(Constants.DriveConstants.kPDriveVel, 0, 0),
          // RamseteCommand passes volts to the callback
          m_drivetrain::tankDriveVolts,
          m_drivetrain);

      // Reset odometry to the starting pose of the trajectory
      m_drivetrain.resetOdometry(trajectory.getInitialPose());

      // Run path following command, then stop at the end.
      return ramseteCommand.andThen(() -> m_drivetrain.tankDriveVolts(0, 0));
    */

  /*       // Create a voltage constraint to ensure we don't accelerate too fast
        var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
                Constants.DriveConstants.kS,
                Constants.DriveConstants.kV,
                Constants.DriveConstants.kA),
            Constants.DriveConstants.kDriveKinematics,
            7);

    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(
                Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(Constants.DriveConstants.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(),
            new Pose2d(3, 0, new Rotation2d(0)),
            config);

    RamseteCommand ramseteCommand =
        new RamseteCommand(
            exampleTrajectory,
            m_drivetrain::getPose,
            new RamseteController(
                Constants.AutoConstants.kRamseteB, Constants.AutoConstants.kRamseteZeta),
            new SimpleMotorFeedforward(
                Constants.DriveConstants.kS,
                Constants.DriveConstants.kV,
                Constants.DriveConstants.kA),
            Constants.DriveConstants.kDriveKinematics,
            m_drivetrain::getWheelSpeeds,
            new PIDController(Constants.DriveConstants.kPDriveVel, 0, 0),
            new PIDController(Constants.DriveConstants.kPDriveVel, 0, 0),
            // RamseteCommand passes volts to the callback
            m_drivetrain::tankDriveVolts,
            m_drivetrain);

    // Reset odometry to starting pose of trajectory.
    m_drivetrain.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> m_drivetrain.tankDriveVolts(0, 0)); */

    //return new TurnAnglePIDCommand(m_drivetrain, 90);
    
    
    // // Simple autonomous test
    //    return new SequentialCommandGroup(//
    // new StartIntakeCommand(m_intake),//
    // new DriveStraightCommand(m_drivetrain, 1.5),//
    // new StopIntakeCommand(m_intake)//
    // );




    // Sequance autonome pour Martin
    return new SequentialCommandGroup(//
            new ParallelCommandGroup(//
              new TrackTargetCommand(m_turret).withTimeout(0.5),//
              new SpinUpShooterCommand(12500, m_shooter).withTimeout(5),//
              new SequentialCommandGroup(//
                new WaitCommand(2.5),
                new RunIndexorCommand(m_indexor).withTimeout(2)//
              )//
            ),//
            //new DriveStraightCommand(m_drivetrain, -1.5),//
            new TurnAnglePIDCommand(m_drivetrain, -190).withTimeout(1),//
            //new WaitCommand(0.5),//
            //new TurnAnglePIDCommand(m_drivetrain, -30),//
            new StartIntakeCommand(m_intake),//
            new ParallelCommandGroup(//
              new DriveStraightCommand(m_drivetrain, 2.0),//
              new SpinIntakeCommand(m_intake),
              new RunConveyorCommand(m_conveyor),
              new RunIndexorColorCommand(m_indexor)).withTimeout(4),//
            //new StopSpinningIntakeCommand(m_intake),//
            new StopIntakeCommand(m_intake),//
            new TurnAnglePIDCommand(m_drivetrain, 190).withTimeout(1),//
            new DriveStraightCommand(m_drivetrain, 2),
            new ParallelCommandGroup(//
              new TrackTargetCommand(m_turret).withTimeout(0.5),//
              new SpinUpShooterCommand(12500, m_shooter).withTimeout(5),//
              new SequentialCommandGroup(//
                new WaitCommand(2.5),//
                new RunIndexorCommand(m_indexor).withTimeout(2)//
              )//
            )//
    );

    //return new SequentialCommandGroup(new DriveStraightReverseCommand(m_drivetrain, -2), new ParallelCommandGroup(new TrackTargetCommand(m_turret), new SpinUpShooterCommand(20000, m_shooter), new SequentialCommandGroup(new WaitCommand(4), new RunIndexorCommand(m_indexor))));


    



  }
  
}
