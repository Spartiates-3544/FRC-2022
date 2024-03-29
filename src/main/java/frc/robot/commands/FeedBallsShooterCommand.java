// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.IndexorSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class FeedBallsShooterCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final IndexorSubsystem indexor;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public FeedBallsShooterCommand(IndexorSubsystem indexor) {
    this.indexor = indexor;
    addRequirements(indexor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("FeedBallsShooterCommand started!");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      indexor.activate(true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("FeedBallsShooterCommand ended!");
      indexor.activate(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
