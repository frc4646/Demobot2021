// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.GamepadDriveTeleOp;

import com.ctre.phoenix.motorcontrol.ControlMode;
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

import frc.robot.PathweaverConstants;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */

  final WPI_VictorSPX frontLeftDrive;
  final WPI_TalonSRX frontRightDrive;
  final WPI_TalonSRX backLeftDrive;
  final WPI_VictorSPX backRightDrive;

  private final Encoder rightEncoder;
  private final Encoder leftEncoder;
  private final double unitsPerRotationMeters;
  private final double kEncoderDistancePerPulseMeters;

  final AHRS navX;

  final PIDController forwardController;
  public double forwardController_kP;
  public double forwardController_kI;
  public double forwardController_kD;
  public double forwardController_tolerance;
  public double forwardController_derivativeTolerance;
  public double forwardController_error;

  final PIDController turnController;
  public double turnController_kP;
  public double turnController_kI;
  public double turnController_kD;
  public double turnController_tolerance;
  public double turnController_derivativeTolerance;
  public double turnController_error;

  private DifferentialDrive differentialDrive;
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
    
    // frontRightDrive.setInverted(true);
    // backRightDrive.setInverted(true);

    leftEncoder = new Encoder(Constants.leftEncoderValues[0], Constants.leftEncoderValues[1]);
    rightEncoder = new Encoder(Constants.rightEncoderValues[0], Constants.rightEncoderValues[1]);

    unitsPerRotationMeters = 0.598f;
    kEncoderDistancePerPulseMeters = unitsPerRotationMeters / 2048;
    leftEncoder.setDistancePerPulse(kEncoderDistancePerPulseMeters);
    rightEncoder.setDistancePerPulse(kEncoderDistancePerPulseMeters);
  
    navX = new AHRS();
    navX.reset();

    forwardController_kP = .1; forwardController_kI = 0; forwardController_kD = 0;
    forwardController_tolerance = 1;
    forwardController_derivativeTolerance = .01;
    forwardController_error = -navX.getRate();
    forwardController = new PIDController(forwardController_kP, forwardController_kI, forwardController_kD);
    forwardController.setTolerance(forwardController_tolerance, forwardController_derivativeTolerance);

    turnController_kP = .1; turnController_kI = 0; turnController_kD = 0;
    turnController_tolerance = 1;
    turnController_derivativeTolerance = .01;
    turnController_error = -navX.getRate();
    turnController = new PIDController(turnController_kP, turnController_kI, turnController_kD);
    turnController.setTolerance(turnController_tolerance, turnController_derivativeTolerance);

    differentialDrive = new DifferentialDrive(backLeftDrive, frontRightDrive);
    
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

  // Drive ///////////////////////////////////////////////////////////////////////////////////////

  public void driveByPercent(double leftSpeed, double rightSpeed)
  {
      //backLeftDrive.set(ControlMode.PercentOutput, leftSpeed);
      //frontRightDrive.set(ControlMode.PercentOutput, rightSpeed);
      differentialDrive.tankDrive(leftSpeed, rightSpeed);
  }

  public void driveByAngle(double targetAngle){
    double calculatedPID = turnController.calculate(getAngle(), targetAngle);
    //frontRightDrive.set(ControlMode.PercentOutput, calculatedPID);
    //backLeftDrive.set(ControlMode.PercentOutput, -calculatedPID);
    differentialDrive.tankDrive(calculatedPID, -calculatedPID);
  }

  public void driveByVolts(double leftVolts, double rightVolts) {
    backLeftDrive.setVoltage(leftVolts);
    frontRightDrive.setVoltage(-rightVolts);
    differentialDrive.feed();
  }

  public void arcadeDrive(double speed, double rotation)
  {
    differentialDrive.arcadeDrive(speed, rotation);
  }

  // Encoders ///////////////////////////////////////////////////////////////////////////////////////

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

  public boolean atTargetDistance()
  {
    return forwardController.atSetpoint();
  }

  public void resetForwardController()
  {
    forwardController.reset();
  }

  public double getLeftTurnRate() {
    return -leftEncoder.getRate();
  }

  public double getRightTurnRate() {
    return -rightEncoder.getRate();
  }

  // Gyro ///////////////////////////////////////////////////////////////////////////////////////

  // For degrees going beyond 360 and below 0
  public double getAngle()
  {
    return navX.getAngle();
  }

  // Restricted between 0 through 360. Used by the NavX.
  public double getHeading() {
    return navX.getRotation2d().getDegrees();
  }

  public void zeroHeading() {
    navX.reset();
  }

  public boolean atTargetAngle()
  {
    return turnController.atSetpoint();
  }

  public void resetTurnController()
  {
    turnController.reset();
  }

  public double getTurnRate() {
    return -navX.getRate();
  }

  // Odometry ///////////////////////////////////////////////////////////////////////////////////////

  public Pose2d getPose(){
    return odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose){
    resetEncoders();
    odometry.resetPosition(pose, navX.getRotation2d());
  }
  
}
