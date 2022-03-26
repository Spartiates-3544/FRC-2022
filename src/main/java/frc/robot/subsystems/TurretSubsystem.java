// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TurretSubsystem extends SubsystemBase {
  private TalonFX turret;
  private DigitalInput limit;

  public TurretSubsystem() {
    turret = new TalonFX(Constants.TurretConstants.TURRETMOTORPORT);
    limit = new DigitalInput(Constants.TurretConstants.LIMITCHANNEL);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (limit.get()) {
      turret.setSelectedSensorPosition(0);
    }
  }


  public void setTurret(double pourcentage) {
    if (pourcentage > 0) {
        //Limit clockwise rotation from the motor
        if (turret.getSelectedSensorPosition() == Constants.TurretConstants.RightLimitTicks) {
            turret.set(TalonFXControlMode.PercentOutput, MathUtil.clamp(pourcentage, 0, 1));
        }
        turret.set(TalonFXControlMode.PercentOutput, pourcentage);
    }

    if (pourcentage < 0) {
        //Limit counterclockwise rotation from the motor
        if (turret.getSelectedSensorPosition() == Constants.TurretConstants.LeftLimitTicks) {
            turret.set(TalonFXControlMode.PercentOutput, MathUtil.clamp(pourcentage, -1, 0));
        }
        turret.set(TalonFXControlMode.PercentOutput, pourcentage);
    }

    turret.set(TalonFXControlMode.PercentOutput, pourcentage);
  }

  public DigitalInput getLimitSwitch() {
      return limit;
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
