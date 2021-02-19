package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

public class PathweaverConstants{
    //Given values
    public static final double ksVolts = 1.07;
    public static final double kvVoltSecondsPerMeter = 1.98;
    public static final double kaVoltSecondsSquaredPerMeter = 0.121;
    public static final double kPDriveVel = 1.99;

    public static final double kTrackwidthMeters = 2.05;
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);
    
    //Our values
    public static final double kMaxSpeedMetersPerSecond = 1.524;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1.524;

    //Default values
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

}