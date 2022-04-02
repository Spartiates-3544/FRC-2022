// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderSimCollection;
import com.kauailabs.navx.frc.AHRS;

import frc.robot.Constants;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
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

    //sim
    private Field2d m_field;
    private TalonFXSimCollection FXsim1;
    private TalonFXSimCollection FXsim2;
    private TalonFXSimCollection FXsim3;
    private TalonFXSimCollection FXsim4;
    private TalonFXSimCollection FXsim5;
    private TalonFXSimCollection FXsim6;
    private CANCoderSimCollection canSimRight;
    private CANCoderSimCollection canSimLeft;
    private DifferentialDrivetrainSim m_driveSim;

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
      gyro = new AHRS(SerialPort.Port.kUSB);
      leftEncoder = new CANCoder(Constants.DriveConstants.LEFTENCODERPORT); 
      rightEncoder = new CANCoder(Constants.DriveConstants.RIGHTENCODERPORT); //TODO CHECK IF I NEED TO REVERSE THE SENSOR DIRECTION https://oldsite.ctr-electronics.com/downloads/api/java/html/classcom_1_1ctre_1_1phoenix_1_1sensors_1_1_c_a_n_coder_configuration.html
      odometry = new DifferentialDriveOdometry(gyro.getRotation2d());
      feedforward = new SimpleMotorFeedforward(Constants.DriveConstants.kS, Constants.DriveConstants.kV, Constants.DriveConstants.kA);
      ramp = new SlewRateLimiter(Constants.DriveConstants.RAMPINGUNITSPERSECOND);
      m_field = new Field2d();
      SmartDashboard.putData("Field", m_field);

      //Sim stuff
      if (RobotBase.isSimulation()) {
        FXsim1 = left1.getSimCollection();
        FXsim2 = left2.getSimCollection();
        FXsim3 = left3.getSimCollection();
        FXsim4 = right1.getSimCollection();
        FXsim5 = right2.getSimCollection();
        FXsim6 = right3.getSimCollection();
        canSimLeft = leftEncoder.getSimCollection();
        canSimRight= rightEncoder.getSimCollection();
  
        // Create the simulation model of our drivetrain.
        m_driveSim = new DifferentialDrivetrainSim(
          DCMotor.getFalcon500(3), // 3 Falcon motors on each side of the drivetrain.
          5.38,                    // 5.38:1 gearing reduction.
          7.5,                     // MOI of 7.5 kg m^2 (from CAD model).
          60.0,                    // The mass of the robot is 60 kg.
          Units.inchesToMeters(3), // The robot uses 3" radius wheels.
          Constants.DriveConstants.kTrackwidthMeters,                  // The track width is 0.4572 meters.
  
          // The standard deviations for measurement noise:
          // x and y:          0.001 m
          // heading:          0.001 rad
          // l and r velocity: 0.1   m/s
          // l and r position: 0.005 m
          VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005));
      }      
  }

  @Override
  public void periodic() {
    odometry.update(gyro.getRotation2d(), getLeftEncoderDistanceMeters(), getRightEncoderDistanceMeters());
    SmartDashboard.putNumber("Angle", gyro.getAngle());
    SmartDashboard.putNumber("Speed", getAverageWheelSpeedsDouble());
    SmartDashboard.putNumber("Distance travelled", getAverageEncoderDistanceMeters());
    m_field.setRobotPose(getPose());
  }

  @Override
  public void simulationPeriodic() {
    FXsim1.setBusVoltage(RobotController.getBatteryVoltage());
    FXsim2.setBusVoltage(RobotController.getBatteryVoltage());
    FXsim3.setBusVoltage(RobotController.getBatteryVoltage());
    FXsim4.setBusVoltage(RobotController.getBatteryVoltage());
    FXsim5.setBusVoltage(RobotController.getBatteryVoltage());
    FXsim6.setBusVoltage(RobotController.getBatteryVoltage());
    m_driveSim.setInputs(left.get() * RobotController.getBatteryVoltage(), right.get() * RobotController.getBatteryVoltage());
    m_driveSim.update(0.02);
    canSimLeft.setRawPosition(distanceToNativeUnits(m_driveSim.getLeftPositionMeters()));
    canSimRight.setRawPosition(distanceToNativeUnits(-m_driveSim.getRightPositionMeters()));
    canSimLeft.setVelocity(velocityToNativeUnits(m_driveSim.getLeftVelocityMetersPerSecond()));
    canSimRight.setVelocity(velocityToNativeUnits(-m_driveSim.getRightVelocityMetersPerSecond()));
    int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
    SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
    SimDouble rate = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Rate"));
    angle.set(m_driveSim.getHeading().getDegrees());
    rate.set(m_driveSim.getHeading().getDegrees() * 50);
  }

  public void arcadeDrive(double fwd, double rot) {
    drive.arcadeDrive(ramp.calculate(fwd), rot);
    drive.feed();
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
    drive.feed();
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
  if (RobotBase.isSimulation()) {
    m_driveSim.setPose(pose);
  }
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

private int distanceToNativeUnits(double positionMeters){
  double wheelRotations = positionMeters/(2 * Math.PI * Units.inchesToMeters(3));
  int sensorCounts = (int)(wheelRotations * 4096);
  return sensorCounts;
}

private int velocityToNativeUnits(double velocityMetersPerSecond){
  double wheelRotationsPerSecond = velocityMetersPerSecond/(2 * Math.PI * Units.inchesToMeters(3));
  double wheelRotationsPer100ms = wheelRotationsPerSecond / 10;
  int sensorCountsPer100ms = (int)(wheelRotationsPer100ms * 4096);
  return sensorCountsPer100ms;
}
}