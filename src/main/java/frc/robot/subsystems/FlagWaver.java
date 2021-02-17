// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Spark;
import frc.robot.Constants;
import frc.robot.commands.FlagStop;

public class FlagWaver extends SubsystemBase {
  /** Creates a new FlagWaver. */

  final Spark flagMotor;
  private final double flagSpeed;

  public FlagWaver() {
    flagMotor = new Spark(Constants.flagMotorPort);
    flagSpeed = 0.25;

    setDefaultCommand(new FlagStop(this));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setMotorSpeed(double speed)
  {
    flagMotor.setSpeed(speed);
  }

  public double getIdealFlagSpeed()
  {
    return flagSpeed;
  }
}
