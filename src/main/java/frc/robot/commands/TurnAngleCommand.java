// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TurnAngleCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DrivetrainSubsystem drivetrain;
  private double angleDegrees;
  private double initialSetpoint;

  public TurnAngleCommand(DrivetrainSubsystem drivetrain, double angleDegrees) {
    this.drivetrain = drivetrain;
    this.angleDegrees = angleDegrees;
    initialSetpoint = angleDegrees;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    angleDegrees += drivetrain.getGyroAngle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      if (initialSetpoint > 0) {
          drivetrain.setMotors(-Constants.AutoConstants.AUTOTURNSPEED, Constants.AutoConstants.AUTOTURNSPEED);
      }
      if (initialSetpoint < 0) {
          drivetrain.setMotors(Constants.AutoConstants.AUTOTURNSPEED, -Constants.AutoConstants.AUTOTURNSPEED);
      }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      drivetrain.setMotors(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (initialSetpoint > 0 && drivetrain.getGyroAngle() >= angleDegrees) {
        return true;
    }
    if (initialSetpoint < 0 && drivetrain.getGyroAngle() <= angleDegrees) {
        return true;
    }
    return false;
  }
}
