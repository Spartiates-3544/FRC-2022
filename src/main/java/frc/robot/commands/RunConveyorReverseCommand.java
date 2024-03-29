// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ConveyorSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.util.RevColorSensor;

public class RunConveyorReverseCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ConveyorSubsystem conveyor;

  public RunConveyorReverseCommand(ConveyorSubsystem conveyor) {
    this.conveyor = conveyor;
    addRequirements(conveyor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      conveyor.setConveyor(-0.4);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      conveyor.setConveyor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
