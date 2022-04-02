// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.TurretSubsystem;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class EncoderSetTurretCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final TurretSubsystem turret;
  private PIDController encoderPID;
  private double setpoint;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public EncoderSetTurretCommand(TurretSubsystem turret, double setpoint) {
    this.turret = turret;
    encoderPID = new PIDController(Constants.TurretConstants.kPEncoder, Constants.TurretConstants.kIEncoder, Constants.TurretConstants.kDEncoder);
    this.setpoint = setpoint;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      encoderPID.setSetpoint(setpoint);
      //encoderPID.setTolerance(positionTolerance, velocityTolerance);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Encoder PID output", MathUtil.clamp(encoderPID.calculate(turret.getTurretEncoder(), setpoint), -1, 1));
    turret.setTurret(MathUtil.clamp(encoderPID.calculate(turret.getTurretEncoder()), -1, 1));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (encoderPID.atSetpoint()) {
        return true;
    }
    return false;
  }
}
