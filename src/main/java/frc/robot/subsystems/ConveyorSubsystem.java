// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ConveyorSubsystem extends SubsystemBase {
  private WPI_TalonFX conveyor;
  public ConveyorSubsystem() {
      conveyor = new WPI_TalonFX(Constants.ConveyorConstants.CONVEYORMOTORPORT);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void setConveyor(double pourcentage) {
      conveyor.set(pourcentage);
      conveyor.feed();
  }

  public void setConveyor(boolean enabled) {
      if (enabled) {
          conveyor.set(0.5);
          conveyor.feed();
      }
      conveyor.set(0);
      conveyor.feed();
  }
}
