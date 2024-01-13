// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CustomTypes.Math.Vector3;

public class Vision extends SubsystemBase {

  Vector3 avgTargetPosInRobotSpace;
  Vector3[] targetPosInRobotSpaceValues = new Vector3[10]; // controls how many values are averaged

  /** Creates a new Vision. */
  public Vision() {}

  @Override
  public void periodic() {
    CalculateTargetPosInRobotSpace();
  }

  void CalculateTargetPosInRobotSpace()
  {
    double[] vals = NetworkTableInstance.getDefault().getTable("limelight").getEntry("targetpose_robotspace").getDoubleArray(new double[6]);
    slideVector3IntoArray(new Vector3(vals[0], vals[1], vals[2]), targetPosInRobotSpaceValues);
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
      out.add(array[i]);
    }

    return out.divideBy(array.length);
  }
}
