// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  private WPI_TalonFX intake;
  private DoubleSolenoid solenoid;

  public IntakeSubsystem() {
    intake = new WPI_TalonFX(Constants.IntakeConstants.INTAKEMOTORPORT);
    solenoid = new DoubleSolenoid(Constants.IntakeConstants.PCMPORT, PneumaticsModuleType.CTREPCM, 0, 1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void deploy(DoubleSolenoid.Value solenoidMode) {
    solenoid.set(solenoidMode);
  }

  public void setIntake(double pourcentage) {
      intake.set(TalonFXControlMode.PercentOutput, pourcentage);
  }

  public void setIntake(boolean on) {
    if (on) {
        intake.set(TalonFXControlMode.PercentOutput, 20);
    }
    intake.set(TalonFXControlMode.PercentOutput, 0);
  }

  public void toggle() {
      solenoid.toggle();
  }
}
