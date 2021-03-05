// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Limelight extends SubsystemBase {
  NetworkTable table;

  NetworkTableEntry tv;
  NetworkTableEntry tx;
  NetworkTableEntry ty;
  NetworkTableEntry ta;

  //Got from Camera
  boolean isTrackingTarget;
  double xOffset;
  double yOffset;
  double area;

  //Given
  double mountingHeight;
  double mountingAngle;
  double targetHeight;

  /** Creates a new Vision. */
  public Limelight() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    tv = table.getEntry("tv");
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");

    mountingHeight = 12; //inches
    mountingAngle = 0;
    targetHeight = 24;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //read values periodically
    if (tv.getNumber(0).intValue() == 1)
    {
      isTrackingTarget = true;
    }
    else
    {
      isTrackingTarget = false;
    }
    xOffset = tx.getDouble(0.0);
    yOffset = ty.getDouble(0.0);
    area = ta.getDouble(0.0);

    //post to smart dashboard periodically
    SmartDashboard.putBoolean("LimelightIsTrackingTarget", isTrackingTarget);
    SmartDashboard.putNumber("LimelightXOffset", xOffset);
    SmartDashboard.putNumber("LimelightYOffset", yOffset);
    SmartDashboard.putNumber("LimelightArea", area);
    SmartDashboard.putNumber("LimelightDistance", getDistanceToTarget());
  }
  public double getDistanceToTarget()
  {
    return (targetHeight-mountingHeight)/Math.tan(Math.toRadians(mountingAngle + yOffset));
  }

  public boolean IsTrackingTarget()
  {
    return isTrackingTarget;
  }

  public double[] TargetPos()
  {
    double[] n = {xOffset, yOffset, area};
    return n;
  }
}
