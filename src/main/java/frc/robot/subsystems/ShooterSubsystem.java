// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
    private WPI_TalonFX upper;
    private WPI_TalonFX lower;
/*     private BangBangController bangbang;
    private SimpleMotorFeedforward feedforward; */

  public ShooterSubsystem() {
      upper = new WPI_TalonFX(Constants.ShooterConstants.UPPERMOTORPORT);
      lower = new WPI_TalonFX(Constants.ShooterConstants.LOWERMOTORPORT);
/*       bangbang = new BangBangController();
      feedforward = new SimpleMotorFeedforward(Constants.ShooterConstants.kS, Constants.ShooterConstants.kV, Constants.ShooterConstants.kA); */
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter velocity", getAverageShooterVelocity());
  }

  public void setMotors(double value) {
    upper.set(value);
    lower.set(value);
  }

  public void setMotorsVoltage(double volts) {
    upper.setVoltage(volts);
    lower.setVoltage(volts);
  }

  public double getAverageShooterVelocity() {
   return ( upper.getSelectedSensorVelocity() + lower.getSelectedSensorVelocity() ) /2;
  }
}
