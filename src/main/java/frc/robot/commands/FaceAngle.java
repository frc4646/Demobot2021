// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class FaceAngle extends CommandBase {
  /** Creates a new FaceAngle. */
  private final Drivetrain m_drivetrain;
  private double targetAngle;

  public FaceAngle(int angle, Drivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    targetAngle = angle;

    m_drivetrain = drivetrain;
    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrain.driveByAngle(targetAngle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_drivetrain.atTargetAngle())
    {
      System.out.println("At Target Angle: Is Finished Returns True");

      return true;
    }
    return false;
  }
}
