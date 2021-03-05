// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FlagWaver;
import frc.robot.subsystems.Limelight;

public class BellSpeedThroughTarget extends CommandBase {
  /** Creates a new BallSpeedThroughTarget. */

  private double minDist = 30;
  private double maxDist = 110;

  private final FlagWaver m_flagWaver;
  private final Limelight m_limelight;

  public BellSpeedThroughTarget(FlagWaver flagWaver, Limelight vision) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_flagWaver = flagWaver;
    m_limelight = vision;

    addRequirements(m_flagWaver);
    addRequirements(m_limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //The range of distances are 30 inches - 110 inches
    if (m_limelight.IsTrackingTarget())
    {
      double dist = m_limelight.getDistanceToTarget();
      if (dist < minDist) dist = minDist;
      else if (dist > maxDist) dist = maxDist;
      double speed = ((dist-minDist)*(100/(maxDist-minDist)))/100; //Cause apparently java doesn't have Math.Clamp()
      speed = 1 - speed; //Close=fast bell Far=Slow Bell
      m_flagWaver.setMotorSpeed(speed);
      System.out.println("Distance: " + dist);
      System.out.println("Flag Speed: " + speed);
    }
    else
    {
      System.out.println("Cannot find target.");
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
    m_flagWaver.setMotorSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
