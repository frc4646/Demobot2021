// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

import frc.robot.PathweaverConstants;

import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import java.util.List;
import java.nio.file.Path;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import java.io.IOException;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.PIDController;


public class FollowTrajectory extends CommandBase {
  /** Creates a new CreateTrajectory. */

  private final Drivetrain m_drivetrain;

 /* DifferentialDriveVoltageConstraint autoVoltageConstraint =
      new DifferentialDriveVoltageConstraint(
          new SimpleMotorFeedforward(PathweaverConstants.ksVolts,
                                      PathweaverConstants.kvVoltSecondsPerMeter,
                                      PathweaverConstants.kaVoltSecondsSquaredPerMeter),
          PathweaverConstants.kDriveKinematics,
          10);

  TrajectoryConfig config =
      new TrajectoryConfig(PathweaverConstants.kMaxSpeedMetersPerSecond,
                            PathweaverConstants.kMaxAccelerationMetersPerSecondSquared)
          // Add kinematics to ensure max speed is actually obeyed
          .setKinematics(PathweaverConstants.kDriveKinematics)
          // Apply the voltage constraint
          .addConstraint(autoVoltageConstraint);
  */
  
  public String trajectoryJSON;
  public Trajectory test1Trajectory;
  public Path trajectoryPath;

  private RamseteCommand ramseteCommand;
  

  public FollowTrajectory(Drivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrain = drivetrain;
    addRequirements(m_drivetrain);


    trajectoryJSON = "paths/test1.wpilib.json";
    trajectoryPath =  Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
    try {
      test1Trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    }
    catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    }

    ramseteCommand = new RamseteCommand(
        test1Trajectory,
        m_drivetrain::getPose,
        new RamseteController(PathweaverConstants.kRamseteB, PathweaverConstants.kRamseteZeta),
        new SimpleMotorFeedforward(PathweaverConstants.ksVolts,
                                   PathweaverConstants.kvVoltSecondsPerMeter,
                                   PathweaverConstants.kaVoltSecondsSquaredPerMeter),
        PathweaverConstants.kDriveKinematics,
        m_drivetrain::getWheelSpeeds,
        new PIDController(PathweaverConstants.kPDriveVel, 0, 0),
        new PIDController(PathweaverConstants.kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback
        m_drivetrain::tankDriveVolts,
        m_drivetrain
    );
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      // Reset odometry to the starting pose of the trajectory.
      m_drivetrain.resetOdometry(test1Trajectory.getInitialPose());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Run path following command, then stop at the end.
    // return ramseteCommand.andThen(() -> m_drivetrain.tankDriveVolts(0, 0));
    return false;
  }
}
