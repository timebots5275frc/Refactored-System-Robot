// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.CustomTypes.Math.Vector3;
import frc.robot.Constants.VisionConstants;

public class Vision extends SubsystemBase {

  private int aprilTagID = -1;
  private double horizontalOffsetFromAprilTag;

  private long valuesGotten = 0; // yucky

  private Vector3 avgAprilTagPosInRobotSpace;
  private Vector3[] AprilTagPosInRobotSpaceValues = new Vector3[VisionConstants.VALUES_TO_AVERAGE];

  private Vector3 avgAprilTagRotInRobotSpace;
  private Vector3[] aprilTagRotInRobotSpaceValues = new Vector3[VisionConstants.VALUES_TO_AVERAGE];

  /** Creates a new Vision. */
  public Vision() 
  {
    VisionDriveCalculator.SetVisionReference(this);

    for(int i = 0; i < VisionConstants.VALUES_TO_AVERAGE; i++)
    {
      AprilTagPosInRobotSpaceValues[i] = Vector3.zero;
      aprilTagRotInRobotSpaceValues[i] = Vector3.zero;
    }
  }

  public void onRobotDisable()
  {
    if (!DriverStation.isAutonomous())
    {
      ToggleLimelightLight(false);
    }
  }

  // called when: test, teleop, auto, and simulation start (in Robot.java)
  public void onRobotEnable()
  {
    ToggleLimelightLight(true);
  }

  @Override
  public void periodic() {
    aprilTagID = (int)NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getDouble(-1.0);
    horizontalOffsetFromAprilTag = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);

    CalculateTargetTransformInRobotSpace();

    LogData();
    valuesGotten++;
  }

  void CalculateTargetTransformInRobotSpace()
  {
    double[] vals = NetworkTableInstance.getDefault().getTable("limelight").getEntry("targetpose_robotspace").getDoubleArray(new double[6]);
    slideVector3IntoArray(new Vector3(vals[0], vals[1], vals[2]), AprilTagPosInRobotSpaceValues);
    slideVector3IntoArray(new Vector3(vals[3], vals[4], vals[5]), aprilTagRotInRobotSpaceValues);
    avgAprilTagPosInRobotSpace = getAverageOfArray(AprilTagPosInRobotSpaceValues);
    avgAprilTagRotInRobotSpace = getAverageOfArray(aprilTagRotInRobotSpaceValues);
  }

  void slideVector3IntoArray(Vector3 newVal, Vector3[] array) // java passes arrays by ref apparrently???
  {
    for (int i = array.length - 2; i > 0; i--)
    {
      array[i] = array[i - 1];
    }

    array[0] = newVal;
  }

  Vector3 getAverageOfArray(Vector3[] array)
  {
    Vector3 out = Vector3.zero;

    for (int i = 0; i < array.length; i++)
    {
      out = out.add(array[i]);
    }

    return out.divideBy(array.length);
  }

  public boolean hasValidData()
  {
    return aprilTagID != -1 && valuesGotten > VisionConstants.VALUES_TO_AVERAGE;
  }

  void LogData()
  {
    SmartDashboard.putString("Detected AprilTagID", aprilTagID == -1 ? "None" : String.valueOf(aprilTagID));
    SmartDashboard.putString("AprilTag position in Robot Space", aprilTagID == -1 ? "N/A" : avgAprilTagPosInRobotSpace.toString(3));
    SmartDashboard.putString("AprilTag rotation in Robot Space", aprilTagID == -1 ? "N/A" : avgAprilTagRotInRobotSpace.toString(3));
    SmartDashboard.putString("AprilTag horizontal offset", aprilTagID == -1 ? "N/A" : String.valueOf(horizontalOffsetFromAprilTag));
  }

  public void ToggleLimelightLight(boolean on)
  {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(on ? 3 : 1);
  }

  // Getter methods //
  public int AprilTagID() { return aprilTagID; }
  public Vector3 AprilTagPosInRobotSpace() { return avgAprilTagPosInRobotSpace; }
  public Vector3 AprilTagRotInRobotSpace() { return avgAprilTagRotInRobotSpace; }
  public double HorizontalOffsetFromAprilTag() { return horizontalOffsetFromAprilTag; }
}
