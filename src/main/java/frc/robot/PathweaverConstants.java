package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

public class PathweaverConstants{
    //Given values
    public static final double ksVolts = 1.09;
    public static final double kvVoltSecondsPerMeter = 1.67;
    public static final double kaVoltSecondsSquaredPerMeter = 0.0799;
    public static final double kPDriveVel = 1.72;

    public static final double kTrackwidthMeters = 2.169;
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);
    
    //Our values
    public static final double kMaxSpeedMetersPerSecond = 6.82f;
    public static final double kMaxAccelerationMetersPerSecondSquared = 2.00f;

    //Default values
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

}