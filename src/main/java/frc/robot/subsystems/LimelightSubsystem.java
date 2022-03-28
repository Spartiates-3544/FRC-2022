// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LimelightSubsystem extends SubsystemBase {
    private double targetHeight;
    private double limelightHeight;
    private double limelightMountingAngle;
    private double hasTarget;
    private double horizontalOffset;
    private double verticalOffset;
    NetworkTable limelight;

  public LimelightSubsystem() {
    limelight = NetworkTableInstance.getDefault().getTable("limelight");
    limelightMountingAngle = Constants.VisionConstants.limelightMountingAngle;
    limelightHeight = Constants.VisionConstants.limelightHeight;
    targetHeight = Constants.VisionConstants.targetHeight;
  }

  @Override
  public void periodic() {
    hasTarget = limelight.getEntry("tv").getDouble(0.0);
    horizontalOffset = limelight.getEntry("tx").getDouble(0.0);
    verticalOffset = limelight.getEntry("ty").getDouble(0.0);
    SmartDashboard.putNumber("Horizontal Offset", horizontalOffset);
    SmartDashboard.putNumber("Vertical Offset", verticalOffset);
    SmartDashboard.putBoolean("Has target", hasTarget());
    SmartDashboard.putNumber("Distance to Hub", getDistanceToHub());
    SmartDashboard.updateValues();
  }

  public boolean hasTarget() {
    if (hasTarget == 0) {
        return false;
    }
    return true;
  }

public double getHorizontalOffset() {
    return horizontalOffset;
}

public double getVerticalOffset() {
    return verticalOffset;
}

public double getDistanceToHub() {
    double angleToGoalRadians = (limelightMountingAngle + verticalOffset) * (3.14159 / 180.0);
    return (targetHeight - limelightHeight) / Math.tan(angleToGoalRadians);
}

public void setCamMode(CamMode mode) {
    switch (mode) {
        case VISION:
            limelight.getEntry("camMode").setNumber(0);
            break;
    
        case DRIVER:
            limelight.getEntry("camMode").setNumber(1);
            break;
    }
  }

  public void setPipeline(int pipeline) {
    limelight.getEntry("pipeline").setNumber(pipeline);
}

public void setStreamMode(StreamMode mode) {
    switch (mode) {
        case STANDARD:
            limelight.getEntry("stream").setNumber(0);
            break;
    
        case PIPMAIN:
            limelight.getEntry("stream").setNumber(1);
            break;

        case PIPSECONDARY:
            limelight.getEntry("stream").setNumber(2);
            break;
    }
  }

  public enum CamMode {
    VISION,
    DRIVER
}  

public enum StreamMode {
    STANDARD,
    PIPMAIN,
    PIPSECONDARY
}
}
