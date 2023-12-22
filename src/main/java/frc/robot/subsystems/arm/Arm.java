// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.ctre.phoenix.sensors.CANCoder;
import com.fasterxml.jackson.databind.introspect.AnnotationCollector.TwoAnnotations;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.Vector;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.CustomTypes.Math.TwoJointInverseKinematics;
import frc.robot.CustomTypes.Math.Vector2;

import java.util.List;
import java.nio.file.FileSystemAlreadyExistsException;
import java.util.ArrayList;

public class Arm extends SubsystemBase {

  // All electronics
  
  private CANSparkMax firstArmController;
  private CANSparkMax secondArmController;
  private RelativeEncoder firstArmEncoder;
  private RelativeEncoder secondArmEncoder;
  private SparkMaxPIDController firstArmPID;
  private SparkMaxPIDController secondArmPID;
  private CANCoder firstArmCANCoder;
  private CANCoder secondArmCANCoder;

  //PID Values

  private double f_kP, f_kI, f_kD, f_kIz, f_kFF, f_kMaxOutput, f_kMinOutput, f_maxRPM, f_smartMAXVelocity,
  f_smartMAXAcc, f_allowedErr, f_minVel;

  private double s_kP, s_kI, s_kD, s_kIz, s_kFF, s_kMaxOutput, s_kMinOutput, s_maxRPM, s_smartMAXVelocity,
  s_smartMAXAcc, s_allowedErr, s_minVel;
  
  public Vector2 armTargetPoint = new Vector2(0, 0); // Change to set default pos
  private double firstArmTargetAngle;
  private double secondArmTargetAngle;
  private double firstArmAngle;
  private double secondArmAngle;
  private TwoJointInverseKinematics kinematics;
  private Joystick joystick;

  public Arm(Joystick joystick) {
    firstArmController = new CANSparkMax(Constants.ArmConstants.FIRST_ARM_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    secondArmController = new CANSparkMax(Constants.ArmConstants.SECOND_ARM_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    firstArmEncoder = firstArmController.getEncoder();
    secondArmEncoder = secondArmController.getEncoder();
    firstArmPID = firstArmController.getPIDController();
    secondArmPID = secondArmController.getPIDController();
    kinematics = new TwoJointInverseKinematics(Constants.ArmConstants.ARM_FIRST_PART_LENGTH, Constants.ArmConstants.ARM_SECOND_PART_LENGTH);
    firstArmCANCoder = new CANCoder(Constants.ArmConstants.FIRST_ARM_CANCODER_ID);
    secondArmCANCoder = new CANCoder(Constants.ArmConstants.SECOND_ARM_CANCODER_ID);
    firstArmEncoder.setPositionConversionFactor(1);
    secondArmEncoder.setPositionConversionFactor(1);
    this.joystick = joystick;


    //PID Values
    f_kP = Constants.ArmConstants.f_kP;
    f_kI = Constants.ArmConstants.f_kI;
    f_kD = Constants.ArmConstants.f_kD;
    f_kIz = Constants.ArmConstants.f_kIz;
    f_kFF = Constants.ArmConstants.f_kFF;
    f_kMaxOutput = Constants.ArmConstants.f_kMaxOutput;
    f_kMinOutput = Constants.ArmConstants.f_kMinOutput;
    f_maxRPM = Constants.ArmConstants.f_maxRPM;
    f_smartMAXVelocity = Constants.ArmConstants.f_smartMAXVelocity;
    f_smartMAXAcc = Constants.ArmConstants.f_smartMAXAcc;
    f_allowedErr = Constants.ArmConstants.f_allowedErr;
    f_minVel = Constants.ArmConstants.f_minVelocity;

    s_kP = Constants.ArmConstants.s_kP;
    s_kI = Constants.ArmConstants.s_kI;
    s_kD = Constants.ArmConstants.s_kD;
    s_kIz = Constants.ArmConstants.s_kIz;
    s_kFF = Constants.ArmConstants.s_kFF;
    s_kMaxOutput = Constants.ArmConstants.s_kMaxOutput;
    s_kMinOutput = Constants.ArmConstants.s_kMinOutput;
    s_maxRPM = Constants.ArmConstants.s_maxRPM;
    s_smartMAXVelocity = Constants.ArmConstants.s_smartMAXVelocity;
    s_smartMAXAcc = Constants.ArmConstants.s_smartMAXAcc;
    s_allowedErr = Constants.ArmConstants.s_allowedErr;
    s_minVel = Constants.ArmConstants.s_minVelocity;

    firstArmPID.setP(f_kP, 0);
    firstArmPID.setI(f_kI, 0);
    firstArmPID.setD(f_kD, 0);
    firstArmPID.setIZone(f_kIz, 0);
    firstArmPID.setFF(f_kFF, 0);
    firstArmPID.setOutputRange(f_kMinOutput, f_kMaxOutput, 0);
    firstArmPID.setSmartMotionMaxVelocity(f_smartMAXVelocity, 0);
    firstArmPID.setSmartMotionMaxAccel(f_smartMAXAcc, 0);
    firstArmPID.setSmartMotionAllowedClosedLoopError(f_allowedErr, 0);
    firstArmPID.setSmartMotionMinOutputVelocity(f_minVel, 0);

    secondArmPID.setP(s_kP, 0);
    secondArmPID.setI(s_kI, 0);
    secondArmPID.setD(s_kD, 0);
    secondArmPID.setIZone(s_kIz, 0);
    secondArmPID.setFF(s_kFF, 0);
    secondArmPID.setOutputRange(s_kMinOutput, s_kMaxOutput, 0);
    secondArmPID.setSmartMotionMaxVelocity(s_smartMAXVelocity, 0);
    secondArmPID.setSmartMotionMaxAccel(s_smartMAXAcc, 0);
    secondArmPID.setSmartMotionAllowedClosedLoopError(s_allowedErr, 0);
    secondArmPID.setSmartMotionMinOutputVelocity(s_minVel, 0);
  }

  private void setArmTargetAngles() {
    firstArmTargetAngle = kinematics.solveFirstJoint(armTargetPoint);
    secondArmTargetAngle = kinematics.solveSecondJoint(armTargetPoint);
  }

  private void setFirstArmAngle() {
    if (firstArmCANCoder.getAbsolutePosition() > -60 && firstArmCANCoder.getAbsolutePosition() <= 180) {
       firstArmAngle = firstArmCANCoder.getAbsolutePosition();
  } else {
       firstArmAngle = firstArmCANCoder.getAbsolutePosition() + 360;/*-firstAngle + ((180 + firstAngle) * 2); */
  }
  }

  public void moveTargetByJoystick(double joystickValue, double joystickValue2) {
    if(Math.abs(joystickValue) > 0.1 || Math.abs(joystickValue2) > 0.1) {
      armTargetPoint.x += joystickValue * Constants.ArmConstants.POINT_MOVEMENT_FACTOR;
      armTargetPoint.y += -joystickValue2 * Constants.ArmConstants.POINT_MOVEMENT_FACTOR;
    }
    
    // Clamping for point
    Vector2 normalizedVector = GetClampedPosValue(armTargetPoint);
    armTargetPoint = normalizedVector;
  }

  private void setSecondArmAngle() {
    secondArmAngle = secondArmCANCoder.getAbsolutePosition();
  }

  public void initializeArm() {
    armTargetPoint = realArmPosition();
  }

  public void moveArm() {
    
    setArmTargetAngles();
    moveArm(firstArmTargetAngle, secondArmTargetAngle);
  }

  public void moveArm(double f_angle, double s_angle) {
    firstArmPID.setReference(f_angle * Constants.ArmConstants.FIRST_ARM_ROTATIONS_PER_DEGREE, CANSparkMax.ControlType.kSmartMotion, 0);
    secondArmPID.setReference(-s_angle * Constants.ArmConstants.SECOND_ARM_ROTATIONS_PER_DEGREE, CANSparkMax.ControlType.kSmartMotion, 0);
  }

  public void setSparkEncoders() {
    setFirstArmAngle();
    setSecondArmAngle();
    firstArmEncoder.setPosition(firstArmAngle * Constants.ArmConstants.FIRST_ARM_ROTATIONS_PER_DEGREE);
    secondArmEncoder.setPosition(-secondArmAngle * Constants.ArmConstants.SECOND_ARM_ROTATIONS_PER_DEGREE);
  }

  public void changeTargetPointByPoint(Vector2 point) {
    Vector2 clampedPoint =  GetClampedPosValue(point);
    armTargetPoint = clampedPoint;
    setArmTargetAngles();
  }


  @Override
  public void periodic() {

    Vector2 silly = realArmPosition();
    double calculatedAngle;
    if (firstArmCANCoder.getAbsolutePosition() > 0 && silly.x < 0) {
      calculatedAngle = firstArmCANCoder.getAbsolutePosition();
    } else {
      calculatedAngle = -firstArmCANCoder.getAbsolutePosition() + ((180 + firstArmCANCoder.getAbsolutePosition()) * 2);
    }
    double adjustedFirstArmAngle;
    if (firstArmCANCoder.getAbsolutePosition() > -60 && firstArmCANCoder.getAbsolutePosition() <= 180) {
      adjustedFirstArmAngle = firstArmCANCoder.getAbsolutePosition();
  } else {
      adjustedFirstArmAngle = firstArmCANCoder.getAbsolutePosition() + 360;/*-firstAngle + ((180 + firstAngle) * 2); */
  }

    // Logging
    
    // SmartDashboard.putNumber("First Arm Mag Encoder", firstArmCANCoder.getAbsolutePosition());
    // SmartDashboard.putNumber("Calculated angle", calculatedAngle);
    // SmartDashboard.putNumber("Actual First Arm Angle", adjustedFirstArmAngle);

    // SmartDashboard.putNumber("Real current rots", firstArmAngle * Constants.ArmConstants.FIRST_ARM_ROTATIONS_PER_DEGREE);
    // SmartDashboard.putNumber("First Arm Spark Encoder", firstArmEncoder.getPosition());
    // SmartDashboard.putNumber("Second Arm Mag Encoder", secondArmCANCoder.getAbsolutePosition());
    // SmartDashboard.putNumber("Second Arm Spark Encoder", secondArmEncoder.getPosition());
    // SmartDashboard.putString("Target Position", armTargetPoint.toString());
    // SmartDashboard.putString("Current Position", realArmPosition().toString());
    // SmartDashboard.putNumber("First Arm target rotations", firstArmTargetAngle * Constants.ArmConstants.FIRST_ARM_ROTATIONS_PER_DEGREE);
    // SmartDashboard.putNumber("Second Arm target rotations", -secondArmTargetAngle * Constants.ArmConstants.SECOND_ARM_ROTATIONS_PER_DEGREE);
    // SmartDashboard.putNumber("First Arm target degrees", firstArmTargetAngle);
    // SmartDashboard.putNumber("Second Arm target degrees", secondArmTargetAngle);
    // SmartDashboard.putNumber("First Arm output current", firstArmController.getOutputCurrent());
    // SmartDashboard.putNumber("Second Arm output current", secondArmController.getOutputCurrent());
    // SmartDashboard.putNumber("First Arm Current Rotations", firstArmEncoder.getPosition());
    // SmartDashboard.putNumber("Second Arm Current Rotations", secondArmEncoder.getPosition());
  }

  public Vector2 GetClampedPosValue(Vector2 pos)
  {
    Vector2 o = pos;
    double armDiff = Math.abs(ArmConstants.ARM_FIRST_PART_LENGTH - ArmConstants.ARM_SECOND_PART_LENGTH);
    double farthestArmReach = kinematics.totalDistance();

    if (o.x < ArmConstants.frontOfBumperXPos) // Inside chassis
    {
        if (o.y > ArmConstants.insideChassisLargestY && o.x < 0) { if (o.y >= ArmConstants.insideChassisLargestY) { o.y = realArmPosition().y; } }
        else if (o.y < ArmConstants.insideChassisSmallestY) { o.y = ArmConstants.insideChassisSmallestY; }
    }
    else // Outside chassis
    {
        double p = PercentBetweenNumbers(o.x, ArmConstants.frontOfBumperXPos, ArmConstants.frontGroundXPos);
        Vector2 point3 = new Vector2(ArmConstants.frontOfBumperXPos, ArmConstants.insideChassisSmallestY);
        Vector2 point4 = new Vector2(ArmConstants.frontGroundXPos, ArmConstants.outsideChassisSmallestY);
        o.y = clampNumber(o.y, Vector2.lerp(point3, point4, p).y, farthestArmReach);

        if (o.y < ArmConstants.outsideChassisSmallestY) { o.y = ArmConstants.outsideChassisSmallestY; }
    }
// cock cum suck 4
    if (o.y >= 0 && o.x < armDiff) { o.x = armDiff; } // Above y=0

    if (o.magnitude() < armDiff + .5f) { o = o.normalized().times(armDiff + .5f); } // Clamp outside min circle
    o = Vector2.clampMagnitude(o, farthestArmReach - .5f); // Clamp inside max circle
    o.x = clampNumber(o.x, ArmConstants.farthestBackChassisPos, ArmConstants.FARTHEST_EXTENSION_POINT); // Clamping between smallest and largest allowed x value

    return o;
  }

  double clampNumber(double val, double min, double max) {
    if (val < min) { return min; }
    else if (val > max) { return max; }
    return val;
  }

  public double PercentBetweenNumbers(double value, double min, double max) {
    double offset = 0 - min;
    return (value + offset) / (max + offset);
  }

  public Vector2 realArmPosition()
  {
    setFirstArmAngle();
    setSecondArmAngle();

    Vector2 firstArmPos = Vector2.RadToVector2(firstArmAngle * ArmConstants.DEG_TO_RAD_RATIO).times(ArmConstants.ARM_FIRST_PART_LENGTH);
    Vector2 secondArmPos = Vector2.RadToVector2((firstArmAngle - secondArmAngle) * ArmConstants.DEG_TO_RAD_RATIO).times(ArmConstants.ARM_SECOND_PART_LENGTH);
    Vector2 thisPos = firstArmPos.add(secondArmPos);
    thisPos.y *= -1;
    return thisPos;
  }
}
