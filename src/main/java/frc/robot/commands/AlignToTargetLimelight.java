// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Drivetrain;
/* Procedure
1) Given: Currently tracking a target
2) Find where the target's position on the x-axis is
3) Drive/rotate to the center
*/

public class AlignToTargetLimelight extends CommandBase {
  /** Creates a new AlignToTarget. */
  private final Limelight m_limelight;
  private final Drivetrain m_drivetrain;
  public AlignToTargetLimelight(Limelight vision, Drivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_limelight = vision;
    m_drivetrain = drivetrain;

    addRequirements(m_limelight);
    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_limelight.IsTrackingTarget()) {
      if (m_limelight.TargetPos()[0] < -3) {
          m_drivetrain.driveByPercent(.2, -.2);
      }      
      else if (m_limelight.TargetPos()[0] > 3) {
          m_drivetrain.driveByPercent(-.2, .2);         
      }
      else {
        end(false);
      }
  }
  else {
    System.out.println("Cannot find target.");
  }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.driveByPercent(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
