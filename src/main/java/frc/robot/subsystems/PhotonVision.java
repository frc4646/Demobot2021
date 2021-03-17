// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.util.Units;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPipelineResult;
import org.photonvision.PhotonTrackedTarget;
import edu.wpi.first.wpilibj.geometry.*;
import java.util.List;

public class PhotonVision extends SubsystemBase {
  /** Creates a new PhotonVision. */
  private final double CAMERA_HEIGHT_METERS;
  private final double TARGET_HEIGHT_METERS;
  private final double CAMERA_PITCH_RADIANS;
  public final double GOAL_RANGE_METERS;

  private PhotonCamera camera;
  private PhotonPipelineResult result;

  private final double LINEAR_P;
  private final double LINEAR_D;
  private PIDController forwardController;

  private final double ANGULAR_P;
  private final double ANGULAR_D;
  private PIDController turnController;

  private double yaw, pitch, area, skew; //Area is scaled from 0-100, by precentage of screen.
  private Transform2d pose;

  public PhotonVision() {
    // Constants such as camera and target height stored. Change per robot and goal!
   CAMERA_HEIGHT_METERS = Units.inchesToMeters(24);
   TARGET_HEIGHT_METERS = Units.feetToMeters(5);
   // Angle between horizontal and the camera.
   CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);
   // How far from the target we want to be
   GOAL_RANGE_METERS = Units.feetToMeters(3);

   // Change this to match the name of your camera
   camera = new PhotonCamera("Microsoft_LifeCam_HD-3000");
   result = camera.getLatestResult();

   // PID constants should be tuned per robot
   LINEAR_P = 0.1;
   LINEAR_D = 0.0;
   forwardController = new PIDController(LINEAR_P, 0, LINEAR_D);

   ANGULAR_P = 0.1;
   ANGULAR_D = 0.0;
   turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);

   camera.setDriverMode(true);
   camera.setPipelineIndex(2);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    result = camera.getLatestResult();

   // Get information from target.
   if (HasTargets())
   {
    yaw = BestTarget().getYaw();
    pitch = BestTarget().getPitch();
    area = BestTarget().getArea();
    skew = BestTarget().getSkew();
    pose = BestTarget().getCameraToTarget();
   }
  }

  public List<PhotonTrackedTarget> Targets()
  {
    return result.getTargets();
  }

  public PhotonTrackedTarget BestTarget()
  {
    return result.getBestTarget();
  }

  public boolean HasTargets()
  {
    return result.hasTargets();
  }

  public int NumberOfTargets()
  {
    return result.getTargets().size();
  }

  public double DistanceToTarget()
  {
    return PhotonUtils.calculateDistanceToTargetMeters(CAMERA_HEIGHT_METERS, TARGET_HEIGHT_METERS, CAMERA_PITCH_RADIANS, 
      Math.toRadians(BestTarget().getPitch()));
  }
  public double DistanceToTarget(PhotonTrackedTarget target)
  {
    return PhotonUtils.calculateDistanceToTargetMeters(CAMERA_HEIGHT_METERS, TARGET_HEIGHT_METERS, CAMERA_PITCH_RADIANS, 
      Math.toRadians(target.getPitch()));
  }
  
  public Translation2d TranslationToTarget()
  {
    return PhotonUtils.estimateCameraToTargetTranslation(
      DistanceToTarget(), Rotation2d.fromDegrees(-BestTarget().getYaw()));
  }
  public Translation2d TranslationToTarget(PhotonTrackedTarget target)
  {
    return PhotonUtils.estimateCameraToTargetTranslation(
      DistanceToTarget(target), Rotation2d.fromDegrees(-target.getYaw()));
  }

  public double TargetPitch()
  {
    return BestTarget().getPitch();
  }
  public double TargetPitch(PhotonTrackedTarget target)
  {
    return target.getPitch();
  }
  
  public double TargetSkew()
  {
    return BestTarget().getSkew();
  }
  public double TargetSkew(PhotonTrackedTarget target)
  {
    return target.getSkew();
  }

  public double TargetYaw()
  {
    return BestTarget().getYaw();
  }
  public double TargetYaw(PhotonTrackedTarget target)
  {
    return target.getYaw();
  }

  public double TargetSize()
  {
    return BestTarget().getArea();
  }
  public double TargetSize(PhotonTrackedTarget target)
  {
    return target.getArea();
  }

  public double latencySeconds()
  {
    return result.getLatencyMillis() / 1000.0;
  }
}
