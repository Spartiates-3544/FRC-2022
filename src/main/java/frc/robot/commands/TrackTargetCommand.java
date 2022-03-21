// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.util.Limelight;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TrackTargetCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final TurretSubsystem turret;
  private final Limelight limelight;
  private double steeringAdjust;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public TrackTargetCommand(TurretSubsystem turret) {
    this.turret = turret;
    limelight = new Limelight();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      if (limelight.hasTarget()) {
          if (limelight.getHorizontalOffset() > 1) {
              steeringAdjust = Constants.TurretConstants.kP * limelight.getHorizontalOffset() - Constants.TurretConstants.minPourcentage;
          } else if (limelight.getHorizontalOffset() < 1) {
              steeringAdjust = Constants.TurretConstants.kP * limelight.getHorizontalOffset() + Constants.TurretConstants.minPourcentage;
          }
          turret.setTurret(steeringAdjust);
      }
      turret.setTurret(0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      turret.setTurret(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
