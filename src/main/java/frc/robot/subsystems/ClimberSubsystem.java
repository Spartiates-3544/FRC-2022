// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
  private WPI_TalonFX leftClimber;
  private WPI_TalonFX rightClimber;

  public ClimberSubsystem() {
      leftClimber = new WPI_TalonFX(Constants.ClimberConstants.LEFTCLIMBERPORT);
      rightClimber = new WPI_TalonFX(Constants.ClimberConstants.RIGHTCLIMBERPORT);
      leftClimber.setInverted(false);
      leftClimber.configForwardSoftLimitThreshold(Constants.ClimberConstants.LEFTREVERSELIMITTHRESHOLD, 0);
      leftClimber.configReverseSoftLimitThreshold(Constants.ClimberConstants.LEFTFWDLIMITTHRESHOLD, 0);
      rightClimber.configReverseSoftLimitThreshold(Constants.ClimberConstants.RIGHTREVERSELIMITTHRESHOLD, 0);
      rightClimber.configForwardSoftLimitThreshold(Constants.ClimberConstants.RIGHTFWDLIMITTHRESHOLD, 0);
      leftClimber.configForwardSoftLimitEnable(false);
      leftClimber.configReverseSoftLimitEnable(false);
      rightClimber.configForwardSoftLimitEnable(true);
      rightClimber.configReverseSoftLimitEnable(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void setClimber(boolean set) {
    if (set) {
        rightClimber.set(0.5);
        leftClimber.set(0.5);
    }
    if (!set) {
        rightClimber.set(0);
        leftClimber.set(0);
    }
  }

  public void setClimber(double pourcentage) {
      rightClimber.set(pourcentage);
      leftClimber.set(pourcentage);
  }

  public double getLeftEncoderPosition() {
      return leftClimber.getSelectedSensorPosition();
  }

  public double getRightEncoderPosition() {
      return rightClimber.getSelectedSensorPosition();
  }
}
