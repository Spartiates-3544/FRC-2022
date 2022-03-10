// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.kauailabs.navx.frc.AHRS;

import frc.robot.Constants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DrivetrainSubsystem extends SubsystemBase {
    private WPI_TalonFX left1;
    private WPI_TalonFX left2;
    private WPI_TalonFX left3;
    private WPI_TalonFX right1;
    private WPI_TalonFX right2;
    private WPI_TalonFX right3;
    private MotorControllerGroup left;
    private MotorControllerGroup right;
    private DifferentialDrive drive;
    private AHRS gyro;
    private CANCoder leftEncoder; //4096 ticks/rotation
    private CANCoder rightEncoder;
    private DifferentialDriveOdometry odometry;
    private SimpleMotorFeedforward feedforward;
    private SlewRateLimiter ramp;

  public DrivetrainSubsystem() {
      left1 = new WPI_TalonFX(Constants.DriveConstants.LEFTDRIVEPORT1);
      left2 = new WPI_TalonFX(Constants.DriveConstants.LEFTDRIVEPORT2);
      left3 = new WPI_TalonFX(Constants.DriveConstants.LEFTDRIVEPORT3);
      right1 = new WPI_TalonFX(Constants.DriveConstants.RIGHTDRIVEPORT1);
      right2 = new WPI_TalonFX(Constants.DriveConstants.RIGHTDRIVEPORT2);
      right3 = new WPI_TalonFX(Constants.DriveConstants.RIGHTDRIVEPORT3);
      left = new MotorControllerGroup(left1, left2, left3);
      right = new MotorControllerGroup(right1, right2, right3);
      right.setInverted(true);
      drive = new DifferentialDrive(left, right);
      gyro = new AHRS(SPI.Port.kMXP);
      leftEncoder = new CANCoder(Constants.DriveConstants.LEFTENCODERPORT); 
      rightEncoder = new CANCoder(Constants.DriveConstants.RIGHTENCODERPORT); //TODO CHECK IF I NEED TO REVERSE THE SENSOR DIRECTION https://oldsite.ctr-electronics.com/downloads/api/java/html/classcom_1_1ctre_1_1phoenix_1_1sensors_1_1_c_a_n_coder_configuration.html
      odometry = new DifferentialDriveOdometry(gyro.getRotation2d());
      drive.setDeadband(0.1); //Deadzone
      feedforward = new SimpleMotorFeedforward(Constants.DriveConstants.kS, Constants.DriveConstants.kV, Constants.DriveConstants.kA);
      ramp = new SlewRateLimiter(Constants.DriveConstants.RAMPINGUNITSPERSECOND);
  }

  @Override
  public void periodic() {
    odometry.update(gyro.getRotation2d(), getLeftEncoderDistanceMeters(), getRightEncoderDistanceMeters());
    SmartDashboard.putData(gyro);
    SmartDashboard.putNumber("Speed", getAverageWheelSpeedsDouble());
    SmartDashboard.putNumber("Distance travelled", getAverageEncoderDistanceMeters());
  }

  public void arcadeDrive(double fwd, double rot) {
    drive.arcadeDrive(fwd, rot);
}

public void setMaxOutput(double maxOutput) {
    drive.setMaxOutput(maxOutput);
  }

public DifferentialDriveWheelSpeeds getWheelSpeeds() {
  return new DifferentialDriveWheelSpeeds(leftEncoder.getVelocity() * 0.00132994089, rightEncoder.getVelocity() * 0.00132994089); //0.00132994089 is the convertion factor for degrees/second (CANCoder default) to meters/second for 6 inch wheels https://www.answers.com/Q/How_do_you_convert_meters_per_second_to_degrees_per_second
}

public double getAverageWheelSpeedsDouble() {
  return ( leftEncoder.getVelocity() * 0.00132994089 + rightEncoder.getVelocity() * 0.00132994089 ) /2;
}

public void setMotors(double leftSpeed, double rightSpeed) {
    left.set(leftSpeed);
    right.set(rightSpeed);
}

public double getEncoderAverageDegrees(){
    return ( leftEncoder.getPosition() + rightEncoder.getPosition() ) /2;
}

public double getAverageEncoderDistanceMeters() {
    return ( getEncoderAverageDegrees() / 360 ) * 0.478778720 /* Circumference of wheels in meters */ ;
}

public double getLeftEncoderDistanceMeters() {
  return ( leftEncoder.getPosition() / 360 ) * 0.478778720 /* Circumference of wheels in meters */ ;
}

public double getRightEncoderDistanceMeters() {
  return ( rightEncoder.getPosition() / 360 ) * 0.478778720 /* Circumference of wheels in meters */ ;
}

public double getGyroAngle() {
    return gyro.getAngle();
}

public void tankDriveVolts(double leftVolts, double rightVolts) {
  left.setVoltage(leftVolts);
  right.setVoltage(rightVolts);
  drive.feed();
}

//Velocity in meters per second
public void tankDriveWithFeedForward(double leftVelocity, double rightVelocity) {
  left.setVoltage(MathUtil.clamp(ramp.calculate(feedforward.calculate(leftVelocity)), 0, 12));
  right.setVoltage(MathUtil.clamp(ramp.calculate(feedforward.calculate(rightVelocity)), 0, 12));
  drive.feed();
}

public double getHeading() {
  return gyro.getRotation2d().getDegrees();
}

public Pose2d getPose() {
  return odometry.getPoseMeters();
}

public void resetOdometry(Pose2d pose) {
  resetEncoders();
  odometry.resetPosition(pose, gyro.getRotation2d());
}

public void resetEncoders() {
  leftEncoder.setPosition(0);
  rightEncoder.setPosition(0);
}

public void zeroHeading() {
    gyro.reset();
}

public CANCoder getLeftEncoder() {
  return leftEncoder;
}

public CANCoder getRightEncoder() {
  return rightEncoder;
}

public double getTurnRate() {
  return -gyro.getRate();
}
}