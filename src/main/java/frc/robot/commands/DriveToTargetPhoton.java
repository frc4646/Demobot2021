// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.controller.PIDController;

import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.Drivetrain;

public class DriveToTargetPhoton extends CommandBase {

  private double rotationSpeed;
  private double forwardSpeed;
  private PIDController turnController;
  private PIDController forwardController;

  private final PhotonVision m_photonvision;
  private final Drivetrain m_drivetrain;
  
  /** Creates a new DriveToTargetPhoton. */
  public DriveToTargetPhoton(PhotonVision vision, Drivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_photonvision = vision;
    m_drivetrain = drivetrain;

    addRequirements(m_photonvision);
    addRequirements(m_drivetrain);

    turnController = new PIDController(0.1f, 0f, 0f);
    forwardController = new PIDController (0.1f, 0f, 0f);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turnController.setTolerance(0.1f);
    forwardController.setTolerance(3f);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_photonvision.HasTargets()) {
      double range = m_photonvision.DistanceToTarget();

      // Use this range as the measurement we give to the PID controller.
      forwardSpeed = forwardController.calculate(range, m_photonvision.GOAL_RANGE_METERS);

      // Also calculate angular power
      rotationSpeed = turnController.calculate(m_photonvision.TargetYaw(), 0);

      System.out.println("Forward speed: " + forwardSpeed);
      System.out.println("Rotation speed: " + rotationSpeed);
    }
    else {
      rotationSpeed = 0f;
      System.out.println("Cannot find target.");
      end(false);
    }
    m_drivetrain.arcadeDrive(forwardSpeed, rotationSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("At setpoint.");
    m_drivetrain.driveByPercent(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return forwardController.atSetpoint();
  }
}
