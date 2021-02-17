// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.GamepadDriveTeleOp;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Encoder;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */

  final VictorSPX frontLeftDrive;
  final TalonSRX frontRightDrive;
  final TalonSRX backLeftDrive;
  final VictorSPX backRightDrive;

  final AHRS navX;
  final PIDController navX_PID;

  private final Encoder rightEncoder;
  private final Encoder leftEncoder;
  private final int encoderCountsPerInch;

  public double navX_kP;
  public double navX_kI;
  public double navX_kD;
  public double navX_tolerance;
  public double navX_derivativeTolerance;
  public double navX_error;

  public Drivetrain() {
    frontLeftDrive = new VictorSPX(Constants.frontLeftDrivePort);
    frontRightDrive = new TalonSRX(Constants.frontRightDrivePort);
    backLeftDrive = new TalonSRX(Constants.backLeftDrivePort);
    backRightDrive = new VictorSPX(Constants.backRightDrivePort);

    //frontLeftDrive.set(ControlMode.Follower, backLeftDrive.getBaseID());
    frontLeftDrive.follow(backLeftDrive);
    //backRightDrive.set(ControlMode.Follower, frontRightDrive.getBaseID());
    backRightDrive.follow(frontRightDrive);
    
    frontRightDrive.setInverted(true);
    backRightDrive.setInverted(true);

    leftEncoder = new Encoder(Constants.leftEncoderValues[0], Constants.leftEncoderValues[1]);
    rightEncoder = new Encoder(Constants.rightEncoderValues[0], Constants.rightEncoderValues[1]);
  
    navX = new AHRS();
    navX.reset();

    navX_kP = .1; navX_kI = 0; navX_kD = 0;
    navX_tolerance = 1;
    navX_derivativeTolerance = .01;
    navX_error = -navX.getRate();

    navX_PID = new PIDController(navX_kP, navX_kI, navX_kD);
    navX_PID.setTolerance(navX_tolerance, navX_derivativeTolerance);

    encoderCountsPerInch = 0;

    setDefaultCommand(new GamepadDriveTeleOp(this));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Left Encoder", leftEncoder.get());
    SmartDashboard.putNumber("Right Encoder", rightEncoder.get());
  }

  public void driveByPercent(double leftSpeed, double rightSpeed)
  {
      frontLeftDrive.set(ControlMode.PercentOutput, rightSpeed);
      backLeftDrive.set(ControlMode.PercentOutput, leftSpeed);
  }

  public double getAngle()
  {
    return navX.getAngle();
  }

  /*public void faceAngle(double targetAngle){
    double error = targetAngle - navX.getAngle();
    frontRightDrive.set(ControlMode.PercentOutput,  kP * error);
    backLeftDrive.set(ControlMode.PercentOutput, -1 * kP * error);
  }*/

  public void faceAngle(double targetAngle){
    double calculatedPID = navX_PID.calculate(getAngle(), targetAngle);
    frontRightDrive.set(ControlMode.PercentOutput, calculatedPID);
    backLeftDrive.set(ControlMode.PercentOutput, -calculatedPID);
  }

  public boolean atTargetAngle()
  {
    return navX_PID.atSetpoint();
  }

  public void resetNavXPID()
  {
    navX_PID.reset();
  }
}
