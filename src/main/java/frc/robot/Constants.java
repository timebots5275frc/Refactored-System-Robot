// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.CustomTypes.PID_Values;

  public final class Constants {
    public static class OperatorConstants {
      public static final int kDriverControllerPort = 0;
    }

    public static final class ControllerConstants {
      public static final int DRIVER_STICK_CHANNEL = 0;
      public static final int AUX_STICK_CHANNEL = 1;
      public static final int XBOXCONTROLLER_CHANNEL = 3;

      public static final double DEADZONE_DRIVE = 0.1;
      public static final double DEADZONE_STEER = 0.3;
    }

    public static final class DriveConstants {

      // Drivetrain Motor IDs
      // There are the CANBus IDs of the SparkMax controllers
      public static final int LEFT_FRONT_DRIVE_MOTOR_ID = 1;
      public static final int LEFT_FRONT_STEER_MOTOR_ID = 2;
      public static final int RIGHT_FRONT_DRIVE_MOTOR_ID = 3;
      public static final int RIGHT_FRONT_STEER_MOTOR_ID = 4;
      public static final int LEFT_REAR_DRIVE_MOTOR_ID = 5;
      public static final int LEFT_REAR_STEER_MOTOR_ID = 6;
      public static final int RIGHT_REAR_DRIVE_MOTOR_ID = 7;
      public static final int RIGHT_REAR_STEER_MOTOR_ID = 8;

      // Drivetrain Encoder IDs
      // These are the CANBus IDs of the CTRE CANCoders
      public static final int LEFT_FRONT_STEER_ENCODER_ID = 10;
      public static final int RIGHT_FRONT_STEER_ENCODER_ID = 11;
      public static final int LEFT_REAR_STEER_ENCODER_ID = 12;
      public static final int RIGHT_REAR_STEER_ENCODER_ID = 13;

      // These constants define the location of the wheels from the center of the
      // robot.
      // These coordinates are determined by the right hand rule.
      // Index finger points in the forward X direction, Thumb points up in the
      // positive Z direction,
      // Middle finger points left in the positive Y direction.

      public static final double LEFT_FRONT_WHEEL_X = (/* inside chassis point */11 - /* wheel center offset */2.25) * 0.0254; // meters
      public static final double LEFT_FRONT_WHEEL_Y = (14 - 2.25) * 0.0254; // meters .5969
      public static final double RIGHT_FRONT_WHEEL_X = (11 - 2.25) * 0.0254; // meters
      public static final double RIGHT_FRONT_WHEEL_Y = (-14 + 2.25) * 0.0254; // meters
      public static final double RIGHT_REAR_WHEEL_X = (-11 + 2.25) * 0.0254; // meters
      public static final double RIGHT_REAR_WHEEL_Y = (-14 + 2.25) * 0.0254; // meters
      public static final double LEFT_REAR_WHEEL_X = (-11 + 2.25) * 0.0254; // meters
      public static final double LEFT_REAR_WHEEL_Y = (14 - 2.25) * 0.0254; // meters

      public static final double WHEEL_RADIUS = 2.0 * 0.0254; // meters * 0.98
      public static final double WHEEL_CIRCUMFERENCE = 2.0 * Math.PI * WHEEL_RADIUS; // meters/revolution

      public static final double MAX_DRIVE_SPEED = 3.5; // meters/second
      public static final double MAX_STEER_RATE = .5; // rotations/second of a wheel for steer.
      public static final double MAX_TWIST_RATE = .6 * 2.0 * Math.PI; // radians/second of the robot rotation.

      // Drive motor gear ratio.
      // | Driving Gear | Driven Gear |
      // First Stage | 14 | 50 |
      // Second Stage | 28 | 16 |
      // Third Stage | 15 | 60 |
      //
      // Overall Gear Ratio = 0.1225
      // One rotation of the motor gives 0.1225 rotations of the wheel.
      // 8.163 rotations of the motor gives one rotation of the wheel.
      public static final double DRIVE_GEAR_RATIO = (14.0 / 50.0) * (28.0 / 16.0) * (15.0 / 60.0);

      // Steer motor gear ratio
      // | Driving Gear | Driven Gear |
      // First Stage | 15 | 32 |
      // Second Stage | 10 | 40 |
      //
      // Overall Gear Ration = 0.1171875
      // One rotation of the motor gives 0.1171875 rotations of the wheel.
      // 8.533 rotations of the motor gives one rotation of the wheel.
      public static final double STEER_GEAR_RATIO = (15.0 / 32) * (10 / 40);

      public static final PID_Values PID_SparkMax_Steer = new PID_Values(0.0001, 0, 0, 0, 0.00005);
      public static final PID_Values PID_Encoder_Steer = new PID_Values(20, 10, 0);

      public static final PID_Values PID_SparkMax_Drive = new PID_Values(0.0003, 0, 0, 0, 0.00016);
      public static final int PIGEON_IMU_ID = 9;
      public static final int PIGEON_2_ID = 9;
  }
}
