// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FlagWaver;

public class FlagWave extends CommandBase {
  /** Creates a new FlagWave. */
  private final FlagWaver m_flagWaver;
  public FlagWave(FlagWaver flagWaver) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_flagWaver = flagWaver;
    addRequirements(m_flagWaver);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_flagWaver.setMotorSpeed(m_flagWaver.getIdealFlagSpeed());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_flagWaver.setMotorSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
