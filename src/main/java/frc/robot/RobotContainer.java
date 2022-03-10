// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IndexorSubsystem;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
import frc.robot.subsystems.ShooterSubsystem;
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DrivetrainSubsystem m_drivetrain = new DrivetrainSubsystem();
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();
  private final IndexorSubsystem m_indexor = new IndexorSubsystem();
  private final XboxController m_controller = new XboxController(0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    m_drivetrain.setDefaultCommand(new DefaultDriveCommand(m_drivetrain, m_controller));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

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
    
  }
  
}
