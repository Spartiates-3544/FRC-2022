package frc.robot.util;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;

//Heavily inspired by https://github.com/BizarbotsRobotics/2022RobotCode
public class Limelight {
    private static final double targetHeight = Constants.VisionConstants.targetHeight;
    private static final double limelightHeight = Constants.VisionConstants.limelightHeight;
    private static final double limelightMountingAngle = Constants.VisionConstants.limelightMountingAngle; //a1

    NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");

    public boolean hasTarget() {
        if (limelight.getEntry("tv").getDouble(0.0) == 0) {
            return false;
        }
        return true;
    }

    public double getHorizontalOffset() {
        return limelight.getEntry("tx").getDouble(0.0);
    }

    public double getVerticalOffset() {
        return limelight.getEntry("ty").getDouble(0.0);
    }

    public double getDistanceToHub() {
        double angleToGoalRadians = Math.toRadians(limelightMountingAngle + limelight.getEntry("ty").getDouble(0.0));
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
