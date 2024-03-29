// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IndexorSubsystem extends SubsystemBase {
  private WPI_TalonFX indexor;

  public IndexorSubsystem() {
      indexor = new WPI_TalonFX(Constants.IndexorConstants.INDEXORMOTORPORT);
  }

  @Override
  public void periodic() {
  }

  public void setPourcentage(double pourcentage) {
    indexor.set(pourcentage);
  }

  public void activate(boolean on) {
    if (on) {
      setPourcentage(30);
    }
    setPourcentage(0);
  }
}
