// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TurretSubsystem extends SubsystemBase {
  private TalonFX turret;
  private DigitalInput rightLimit;
  private DigitalInput leftLimit;

  public TurretSubsystem() {
    turret = new TalonFX(Constants.TurretConstants.TURRETMOTORPORT);
    rightLimit = new DigitalInput(Constants.TurretConstants.RIGHTLIMITCHANNEL);
    leftLimit = new DigitalInput(Constants.TurretConstants.LEFTLIMITCHANNEL);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


  public void setTurret(double pourcentage) {
    if (pourcentage > 0) {
        //Limit clockwise rotation from the motor
        if (rightLimit.get()) {
            turret.set(TalonFXControlMode.PercentOutput, 0);
        }
        turret.set(TalonFXControlMode.PercentOutput, pourcentage);
    }

    if (pourcentage < 0) {
        //Limit counterclockwise rotation from the motor
        if (leftLimit.get()) {
            turret.set(TalonFXControlMode.PercentOutput, 0);
        }
        turret.set(TalonFXControlMode.PercentOutput, pourcentage);
    }

    turret.set(TalonFXControlMode.PercentOutput, pourcentage);
  }

  public DigitalInput getRightLimitSwitch() {
      return rightLimit;
  }

  public DigitalInput getLeftLimitSwitch() {
      return leftLimit;
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
