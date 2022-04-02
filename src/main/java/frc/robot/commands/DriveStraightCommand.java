// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ExampleSubsystem;

import java.io.Console;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveStraightCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DrivetrainSubsystem drivetrain;
  private double distanceMeters;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveStraightCommand(DrivetrainSubsystem drivetrain, double distanceMeters) {
    this.drivetrain = drivetrain;
    this.distanceMeters = distanceMeters;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    distanceMeters += drivetrain.getAverageEncoderDistanceMeters();
    System.out.println("DriveStraightCommand started!");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      drivetrain.setMotors(-Constants.AutoConstants.AUTOFWDSPEED, -Constants.AutoConstants.AUTOFWDSPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      System.out.println("DriveStraightCommand ended!");
      drivetrain.setMotors(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (drivetrain.getAverageEncoderDistanceMeters() > distanceMeters) {
        return true;
    }
    return false;
  }
}
