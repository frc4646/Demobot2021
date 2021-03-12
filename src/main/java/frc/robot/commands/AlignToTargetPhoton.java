// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.controller.PIDController;

import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.Drivetrain;

public class AlignToTargetPhoton extends CommandBase {
  /** Creates a new AlignToTargetPhoton. */

  private double rotationSpeed;
  private PIDController controller;

  private final PhotonVision m_photonvision;
  private final Drivetrain m_drivetrain;
  public AlignToTargetPhoton(PhotonVision vision, Drivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_photonvision = vision;
    m_drivetrain = drivetrain;

    addRequirements(m_photonvision);
    addRequirements(m_drivetrain);

    controller = new PIDController(0.1f, 0f, 0f);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    controller.setTolerance(0.1f);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_photonvision.HasTargets()) {
      rotationSpeed = controller.calculate(m_photonvision.TargetYaw(), 0);
      System.out.println("Rotation speed: " + rotationSpeed);
    }
    else {
      rotationSpeed = 0f;
      System.out.println("Cannot find target.");
      end(false);
    }
    m_drivetrain.driveByPercent(rotationSpeed, -rotationSpeed);
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
    return controller.atSetpoint();
  }
}
