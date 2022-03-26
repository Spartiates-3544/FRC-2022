// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.CommandBase;

/* Regular command for driving */
public class DefaultDriveCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DrivetrainSubsystem m_drivetrain;
  private XboxController m_controller;

  public DefaultDriveCommand(DrivetrainSubsystem drivetrain, XboxController controller) {
    m_drivetrain = drivetrain;
    m_controller = controller;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    m_drivetrain.setMaxOutput(0.5);
  }

  @Override
  public void execute() {   
        m_drivetrain.arcadeDrive(m_controller.getLeftY(), m_controller.getLeftTriggerAxis() - m_controller.getRightTriggerAxis());
        /*
        var speeds = DifferentialDrive.arcadeDriveIK(m_controller.getLeftY(), m_controller.getLeftTriggerAxis() - m_controller.getRightTriggerAxis(), true);
        m_drivetrain.tankDriveWithFeedForward(speeds.left * Constants.DriveConstants.maxMetersPerSecond, speeds.right * Constants.DriveConstants.maxMetersPerSecond);
        */
  } 

  @Override
  public void end(boolean interrupted) {
      m_drivetrain.setMotors(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
