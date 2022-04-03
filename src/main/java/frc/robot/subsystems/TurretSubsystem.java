// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class TurretSubsystem extends SubsystemBase {
  private WPI_TalonFX turret;
  private DigitalInput limit;
  /*
  private PIDController limelightPid = new PIDController(Constants.TurretConstants.kP, Constants.TurretConstants.kI, Constants.TurretConstants.kD);
  private PIDController encoderPid = new PIDController(Constants.TurretConstants.kPEncoder, Constants.TurretConstants.kIEncoder, Constants.TurretConstants.kDEncoder);
  private double encoderValue;
  */

  public TurretSubsystem() {
    turret = new WPI_TalonFX(Constants.TurretConstants.TURRETMOTORPORT);
    turret.setSelectedSensorPosition(0.0); //-148 000 a 148 000
    limit = new DigitalInput(Constants.TurretConstants.LIMITCHANNEL);
    turret.configForwardSoftLimitThreshold(Constants.TurretConstants.FWDLIMITTHRESHOLD, 0);
    turret.configReverseSoftLimitThreshold(Constants.TurretConstants.REVERSELIMITTHRESHOLD, 0);
    turret.configForwardSoftLimitEnable(true, 0);
    turret.configReverseSoftLimitEnable(true, 0);
    //encoderValue = turret.getSelectedSensorPosition();
    // encoderPid.enableContinuousInput(minimumInput, maximumInput);
    //encoderPid.disableContinuousInput();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Turret encoder value", turret.getSelectedSensorPosition(0));
  }

  /*
  public void encoderAutoSet(double setpoint) {
    //-148000 to 148000
    //turret.set(TalonFXControlMode.PercentOutput, encoderPid.calculate(encoderValue, setpoint));
    SmartDashboard.putNumber("Encoder PID output", MathUtil.clamp(encoderPid.calculate(turret.getSelectedSensorPosition(), setpoint), -1, 1));
    turret.set(TalonFXControlMode.PercentOutput, MathUtil.clamp(encoderPid.calculate(turret.getSelectedSensorPosition(), setpoint), -1, 1));
  }
  */

  /*
  public void limelightAutoSet(double setpoint) {
    SmartDashboard.putNumber("Limelight PID output", MathUtil.clamp(limelightPid.calculate(limelight.getHorizontalOffset(), setpoint), -1, 1));
    turret.set(TalonFXControlMode.PercentOutput, MathUtil.clamp(limelightPid.calculate(limelight.getHorizontalOffset(), setpoint), -1, 1));
  }
  */

  public void setTurret(double pourcentage) {
    turret.set(TalonFXControlMode.PercentOutput, pourcentage);
    turret.feed();
  }

  public DigitalInput getLimitSwitch() {
      return limit;
  }

  public double getTurretEncoder() {
    return turret.getSelectedSensorPosition();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
