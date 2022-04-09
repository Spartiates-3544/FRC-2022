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
        public static int LEFTDRIVEPORT1 = 7;
        public static int LEFTDRIVEPORT2 = 2;
        public static int LEFTDRIVEPORT3 = 6;
        public static int RIGHTDRIVEPORT1 = 3;
        public static int RIGHTDRIVEPORT2 = 10;
        public static int RIGHTDRIVEPORT3 = 5;
        public static int LEFTENCODERPORT = 0;
        public static int RIGHTENCODERPORT = 1;
        public static double RAMPINGUNITSPERSECOND = 1.2;
        public static double kP = 0.003;
        public static double kI = 0;
        public static double kD = 0;
        public static double kTrackwidthMeters = 0.4572;
        public static DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters); //TODO Set all of the above
        public static double kPDriveVel = 2; //TODO
        public static double maxMetersPerSecond = 5;
    }

    public final static class IndexorConstants {
        public static int INDEXORMOTORPORT = 13;
    }

    public final static class ShooterConstants {
        public static int LOWERMOTORPORT = 9;
        public static int UPPERMOTORPORT = 4;
        public static double kPUpper = 0.00070685;
        public static double kPLower = 0.00049724;
        public static double feedforward = 0.00005;
        public static double distanceFeedforward;
        public static double kP = 0.00002;
    }

    public final static class AutoConstants {
        public static double kMaxSpeedMetersPerSecond = 5; //TODO
        public static double kMaxAccelerationMetersPerSecondSquared = 2; //TODO
        public static double AUTOFWDSPEED = 0.15;

        public static double AUTOTURNSPEED = 0.20;
        public static String trajectoryName = "test2"; //Change as needed 
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
    }

    public final static class VisionConstants {
        public static double limelightHeight = 32.875;
        public static double targetHeight = 104;
        public static double limelightMountingAngle = 26.5;
    }

    //TODO
    public final static class TurretConstants {
        public static final double FWDLIMITTHRESHOLD = 40000;
        public static final double REVERSELIMITTHRESHOLD = -40000;
        public static int TURRETMOTORPORT = 8;
        public static int LIMITCHANNEL = 0;
        public static double kP = 0.02;
        public static double kI = 0;
        public static double kD = 0;
        public static double kPEncoder = 0.0001;
        public static double kIEncoder = 0;
        public static double kDEncoder = 0;
        public static double minPourcentage = 0.02;
        public static double RightLimitTicks = 0; //TODO
        public static double LeftLimitTicks = 0; //TODO
    }

    public final static class IntakeConstants {
        public static double INTAKESPEED = -0.6;
		public static int INTAKEMOTORPORT = 11; 
        public static int PCMPORT = 14;
    }

    public final static class ClimberConstants {
        public static double RIGHTREVERSELIMITTHRESHOLD = 0;
        public static double RIGHTFWDLIMITTHRESHOLD = 262000;
        public static double LEFTREVERSELIMITTHRESHOLD = -262000;
        public static double LEFTFWDLIMITTHRESHOLD = 0;
        public static int LEFTCLIMBERPORT = 16;
        public static int RIGHTCLIMBERPORT = 17;
    }

    public final static class ConveyorConstants {
        public static int CONVEYORMOTORPORT = 15;
    }
}
