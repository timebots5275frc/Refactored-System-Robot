// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.DriveTrain;

import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.CustomTypes.Abstraction.Gyroscope;
import frc.robot.CustomTypes.Abstraction.AbstractedGyroscopes.AbstractPigeon2;

public class SwerveDrive extends SubsystemBase {

  private final Translation2d leftFrontWheelLocation = new Translation2d(DriveConstants.LEFT_FRONT_WHEEL_X, DriveConstants.LEFT_FRONT_WHEEL_Y);
  private final Translation2d rightFrontWheelLocation = new Translation2d(DriveConstants.RIGHT_FRONT_WHEEL_X, DriveConstants.RIGHT_FRONT_WHEEL_Y);    
  private final Translation2d rightRearWheelLocation = new Translation2d(DriveConstants.RIGHT_REAR_WHEEL_X, DriveConstants.RIGHT_REAR_WHEEL_Y);
  private final Translation2d leftRearWheelLocation = new Translation2d(DriveConstants.LEFT_REAR_WHEEL_X, DriveConstants.LEFT_REAR_WHEEL_Y);

  private final SwerveModule leftFrontSwerveModule = new SwerveModule(DriveConstants.LEFT_FRONT_DRIVE_MOTOR_ID, DriveConstants.LEFT_FRONT_STEER_MOTOR_ID, DriveConstants.LEFT_FRONT_STEER_ENCODER_ID);
  private final SwerveModule rightFrontSwerveModule = new SwerveModule(DriveConstants.RIGHT_FRONT_DRIVE_MOTOR_ID, DriveConstants.RIGHT_FRONT_STEER_MOTOR_ID, DriveConstants.RIGHT_FRONT_STEER_ENCODER_ID);
  private final SwerveModule rightRearSwerveModule = new SwerveModule(DriveConstants.RIGHT_REAR_DRIVE_MOTOR_ID, DriveConstants.RIGHT_REAR_STEER_MOTOR_ID, DriveConstants.RIGHT_REAR_STEER_ENCODER_ID);
  private final SwerveModule leftRearSwerveModule = new SwerveModule(DriveConstants.LEFT_REAR_DRIVE_MOTOR_ID, DriveConstants.LEFT_REAR_STEER_MOTOR_ID, DriveConstants.LEFT_REAR_STEER_ENCODER_ID);
  
  Gyroscope gyroscope = new AbstractPigeon2(new Pigeon2(DriveConstants.PIGEON_2_ID));

  public final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(leftFrontWheelLocation, rightFrontWheelLocation, rightRearWheelLocation, leftRearWheelLocation);
  public final SwerveModulePosition[] modulePositions = new SwerveModulePosition[] {leftFrontSwerveModule.getPosition(), rightFrontSwerveModule.getPosition(), rightRearSwerveModule.getPosition(), leftFrontSwerveModule.getPosition()};
  private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(kinematics, this.getHeading(), modulePositions);

  public SwerveDrive() 
  {

  }

  @Override
  public void periodic() {

  }

  public Rotation2d getHeading() {

    Rotation2d heading = Rotation2d.fromDegrees(gyroscope.getYaw());


    // System.out.println( "getYComplementaryAngle = " + heading );
    // System.out.println( "getXComplementaryAngle = " +
    // imuADIS16470.getXComplementaryAngle() );

    return heading; // TODO Lucas //.minus(new Rotation2d(this.autoTurnOffsetRadians)); // radians

}
}
