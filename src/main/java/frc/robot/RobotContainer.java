// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.DriveStraightCommand;
import frc.robot.commands.FeedBallsShooterCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.RunIndexorCommand;
import frc.robot.commands.SpinUpShooterCommand;
import frc.robot.commands.TurnAngleCommand;
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

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    m_drivetrain.setDefaultCommand(new DefaultDriveCommand(m_drivetrain, m_controller));
    //m_intake.setDefaultCommand(new IntakeCommand(m_intake));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /*
    new JoystickButton(m_controller, Button.kA.value).toggleWhenPressed(
        new ParallelCommandGroup(new SpinUpShooterCommand(10, m_shooter), //TODO make the velocity decision automatic
        new SequentialCommandGroup(new WaitCommand(2.5), new FeedBallsShooterCommand(m_indexor))), 
        true);
    */
    new JoystickButton(m_controller, Button.kB.value).toggleWhenPressed(new IntakeCommand(m_intake), true);
    new JoystickButton(m_controller, Button.kX.value).toggleWhenPressed(new SpinUpShooterCommand(9624, m_shooter));
    new JoystickButton(m_controller, Button.kA.value).toggleWhenPressed(new RunIndexorCommand(m_indexor), true);
    new JoystickButton(m_controller, Button.kY.value).toggleWhenPressed(new ParallelCommandGroup(new IntakeCommand(m_intake), new SpinUpShooterCommand(15000, m_shooter), new RunIndexorCommand(m_indexor)));

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

    return new TurnAngleCommand(m_drivetrain, 90);


  }
  
}
