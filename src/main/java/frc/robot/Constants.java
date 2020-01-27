/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class Chassis {
        public final static int kLeftTopMotor = 2;
        public final static int kLeftFrontMotor = 3;
        public final static int kLeftBackMotor = 1;
        public final static int kRightTopMotor = 5;
        public final static int kRightFrontMotor = 4;
        public final static int kRightBackMotor = 6;

        public final static int[] kRightEncoderPorts = {-1}; //TODO
        public final static boolean kRightEncoderReversed = false;
        public final static int[] kLeftEncoderPorts = {-1}; //TODO
        public final static boolean kLeftEncoderReversed = false; //TODO

        public final static double kEncoderDistancePerPulse = -1; //TODO
        public final static boolean kGyroReversed = false;
    }

    public static final class OI {
        public final static int kController = 0;

    }

    public static final class Robotpath {
        // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
        // These characterization values MUST be determined either experimentally or
        // theoretically
        // for *your* robot's drive.
        // The Robot Characterization Toolsuite provides a convenient tool for obtaining
        // these
        // values for your robot.
        public static final double ksVolts = -1; //TODO
        public static final double kvVoltSecondsPerMeter = -1; //TODO
        public static final double kaVoltSecondsSquaredPerMeter = -1; //TODO

        // Example value only - as above, this must be tuned for your drive!
        public static final double kPDriveVel = -1;//TODO
        public static final double kTrackwidthMeters = -1;//TODO
        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
                kTrackwidthMeters);

        public static final double kMaxSpeedMetersPerSecond = -1;//TODO
        public static final double kMaxAccelerationMetersPerSecondSquared = -1;//TODO

        // Reasonable baseline values for a RAMSETE follower in units of meters and
        // seconds
        public static final double kRamseteB = -1;//TODO
        public static final double kRamseteZeta = -1;//TODO
    }
}
