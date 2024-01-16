// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CustomTypes.Math.Vector3;

public class Vision extends SubsystemBase {

  private int aprilTagID = -1;
  private double horizontalOffsetFromTarget;

  private final int valuesToAvg = 3;
  private long valuesGotten = 0; // yucky

  private Vector3 avgTargetPosInRobotSpace;
  private Vector3[] targetPosInRobotSpaceValues = new Vector3[valuesToAvg]; // controls how many values are averaged

  /** Creates a new Vision. */
  public Vision() 
  {
    VisionDriveCalculator.SetVisionReference(this);

    for(int i = 0; i < valuesToAvg; i++)
    {
      targetPosInRobotSpaceValues[i] = Vector3.zero;
    }
  }

  @Override
  public void periodic() {
    aprilTagID = (int)NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getDouble(-1.0);
    horizontalOffsetFromTarget = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);

    CalculateTargetPosInRobotSpace();

    LogData();
    valuesGotten++;
  }

  void CalculateTargetPosInRobotSpace()
  {
    double[] vals = NetworkTableInstance.getDefault().getTable("limelight").getEntry("targetpose_robotspace").getDoubleArray(new double[6]);
    slideVector3IntoArray(new Vector3(vals[0], vals[2], vals[4]), targetPosInRobotSpaceValues);
    avgTargetPosInRobotSpace = getAverageOfArray(targetPosInRobotSpaceValues);
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
    return aprilTagID != -1 && valuesGotten > valuesToAvg;
  }

  void LogData()
  {
    SmartDashboard.putString("Detected AprilTagID", aprilTagID == -1 ? "None" : String.valueOf(aprilTagID));
    SmartDashboard.putString("Target position in Robot Space", aprilTagID == -1 ? "N/A" : avgTargetPosInRobotSpace.toString());
    SmartDashboard.putString("Target horizontal offset", aprilTagID == -1 ? "N/A" : String.valueOf(horizontalOffsetFromTarget));
  }

  // Getter methods //
  public int AprilTagID() { return aprilTagID; }
  public Vector3 TargetPosInRobotSpace() { return avgTargetPosInRobotSpace; }
  public double HorizontalOffsetFromTarget() { return horizontalOffsetFromTarget; }
}
