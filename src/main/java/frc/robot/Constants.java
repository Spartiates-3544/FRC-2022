// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public final static class DriveConstants {
        public static int LEFTDRIVEPORT1 = 2;
        public static int LEFTDRIVEPORT2 = 6;
        public static int LEFTDRIVEPORT3 = 7;
        public static int RIGHTDRIVEPORT1 = 3;
        public static int RIGHTDRIVEPORT2 = 4;
        public static int RIGHTDRIVEPORT3 = 5;
        public static int LEFTENCODERPORT = 0; //TODO CHANGE
        public static int RIGHTENCODERPORT = 0; //TODO CHANGE
        public static double RAMPINGUNITSPERSECOND = 3; //TODO TEST
        public static double kS = 0;
        public static double kV = 0;
        public static double kA = 0;
        public static double kTrackwidthMeters = 0;
        public static DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters); //TODO Set all of the above
        public static double kPDriveVel = 0; //TODO
        public static double maxMetersPerSecond = 5;
    }

    public final static class IndexorConstants {
        public static int INDEXORMOTORPORT = 0; //TODO
    }

    public final static class ShooterConstants {
        public static int LOWERMOTORPORT = 0; //TODO
        public static int UPPERMOTORPORT = 0; //TODO
        public static double kS = 0; //TODO
        public static double kV = 0; //TODO to calculate values: https://www.reca.lc/flywheel
        public static double kA = 0; //TODO
    }

    public final static class AutoConstants {
        public static double kMaxSpeedMetersPerSecond = 0; //TODO
        public static double kMaxAccelerationMetersPerSecondSquared = 0; //TODO
        public static String trajectoryName = "test"; //Change as needed 
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
    }

    public final static class VisionConstants {
        public static double limelightHeight = 0; //TODO
        public static double targetHeight = 104;
        public static double limelightMountingAngle = 0; //TODO
    }
}
