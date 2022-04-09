// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RetractClimberCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ClimberSubsystem climber;

  public RetractClimberCommand(ClimberSubsystem climber) {
    this.climber = climber;
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      climber.setClimber(-0.7);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      climber.setClimber(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (climber.getRightEncoderPosition() <= Constants.ClimberConstants.RIGHTREVERSELIMITTHRESHOLD) {
        return true;
    }
    return false;
  }
}
