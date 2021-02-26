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
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Encoder;

import edu.wpi.first.wpilibj.trajectory.Trajectory;

import java.io.IOException;
import java.nio.file.Path;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import frc.robot.PathweaverConstants;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */

  final WPI_VictorSPX frontLeftDrive;
  final WPI_TalonSRX frontRightDrive;
  final WPI_TalonSRX backLeftDrive;
  final WPI_VictorSPX backRightDrive;

  final AHRS navX;
  final PIDController navX_PID;

  private final Encoder rightEncoder;
  private final Encoder leftEncoder;
  private final int encoderCountsPerInch;
  private final double unitsPerRotationMeters;
  private final double kEncoderDistancePerPulseMeters;

  public double navX_kP;
  public double navX_kI;
  public double navX_kD;
  public double navX_tolerance;
  public double navX_derivativeTolerance;
  public double navX_error;

  private DifferentialDriveOdometry odometry;

  public Drivetrain() {
    frontLeftDrive = new WPI_VictorSPX(Constants.frontLeftDrivePort);
    frontRightDrive = new WPI_TalonSRX(Constants.frontRightDrivePort);
    backLeftDrive = new WPI_TalonSRX(Constants.backLeftDrivePort);
    backRightDrive = new WPI_VictorSPX(Constants.backRightDrivePort);

    //frontLeftDrive.set(ControlMode.Follower, backLeftDrive.getBaseID());
    frontLeftDrive.follow(backLeftDrive);
    //backRightDrive.set(ControlMode.Follower, frontRightDrive.getBaseID());
    backRightDrive.follow(frontRightDrive);
    
    frontRightDrive.setInverted(true);
    backRightDrive.setInverted(true);

    leftEncoder = new Encoder(Constants.leftEncoderValues[0], Constants.leftEncoderValues[1]);
    rightEncoder = new Encoder(Constants.rightEncoderValues[0], Constants.rightEncoderValues[1]);

    unitsPerRotationMeters = 0.598f;
    kEncoderDistancePerPulseMeters = unitsPerRotationMeters / 2048;
    leftEncoder.setDistancePerPulse(kEncoderDistancePerPulseMeters);
    rightEncoder.setDistancePerPulse(kEncoderDistancePerPulseMeters);
  
    navX = new AHRS();
    navX.reset();

    navX_kP = .1; navX_kI = 0; navX_kD = 0;
    navX_tolerance = 1;
    navX_derivativeTolerance = .01;
    navX_error = -navX.getRate();

    navX_PID = new PIDController(navX_kP, navX_kI, navX_kD);
    navX_PID.setTolerance(navX_tolerance, navX_derivativeTolerance);

    encoderCountsPerInch = 0;
    
    odometry = new DifferentialDriveOdometry(navX.getRotation2d());

    setDefaultCommand(new GamepadDriveTeleOp(this));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Left Encoder", leftEncoder.get());
    SmartDashboard.putNumber("Right Encoder", rightEncoder.get());

    odometry.update(navX.getRotation2d(), leftEncoder.getDistance(), rightEncoder.getDistance());
  }

  // Motors

  public void driveByPercent(double leftSpeed, double rightSpeed)
  {
      backLeftDrive.set(ControlMode.PercentOutput, leftSpeed);
      frontRightDrive.set(ControlMode.PercentOutput, rightSpeed);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    backLeftDrive.setVoltage(leftVolts);
    frontRightDrive.setVoltage(rightVolts);
  }

  // Encoders

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate());
  }

  public double getAverageEncoderDistance() {
    return (leftEncoder.getDistance() + rightEncoder.getDistance()) / 2.0;
  }

  public void resetEncoders()
  {
    leftEncoder.reset();
    rightEncoder.reset();
  }

  // Gyro

  // For degrees going beyond 360 and below 0
  public double getAngle()
  {
    return navX.getAngle();
  }

  // Restricted between 0 through 360. Used by the NavX.
  public double getHeading() {
    return navX.getRotation2d().getDegrees();
  }

  public void faceAngle(double targetAngle){
    double calculatedPID = navX_PID.calculate(getAngle(), targetAngle);
    frontRightDrive.set(ControlMode.PercentOutput, calculatedPID);
    backLeftDrive.set(ControlMode.PercentOutput, -calculatedPID);
  }

  public void zeroHeading() {
    navX.reset();
  }

  public boolean atTargetAngle()
  {
    return navX_PID.atSetpoint();
  }

  public void resetNavXPID()
  {
    navX_PID.reset();
  }

  public double getTurnRate() {
    return -navX.getRate();
  }

  // Odometry

  public Pose2d getPose(){
    return odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose){
    resetEncoders();
    odometry.resetPosition(pose, navX.getRotation2d());
  }
  
}
