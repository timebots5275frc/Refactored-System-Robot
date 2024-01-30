// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.CustomTypes.Math.Vector2;
import frc.robot.commands.Autos;
import frc.robot.commands.TeleopJoystickDrive;
import frc.robot.commands.AutoVisionDrive;
import frc.robot.subsystems.DriveTrain.SwerveDrive;
import frc.robot.subsystems.Vision.Vision;
import frc.robot.subsystems.Vision.VisionFollowAprilTag;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {

  SwerveDrive swerveDrive = new SwerveDrive();
  Vision vision = new Vision();
  Joystick driveJoystick = new Joystick(0);
  Joystick sillyJoystick = new Joystick(1);

  TeleopJoystickDrive teleopJoystickDriveCommand = new TeleopJoystickDrive(swerveDrive, driveJoystick, true);

  AutoVisionDrive visionDrivePoint1 = new AutoVisionDrive(swerveDrive, vision, new Vector2(-.5, 1));
  AutoVisionDrive visionDrivePoint2 = new AutoVisionDrive(swerveDrive, vision, new Vector2(.5, 1));
  AutoVisionDrive visionDrivePoint3 = new AutoVisionDrive(swerveDrive, vision, new Vector2(0, 2.5));

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    swerveDrive.setDefaultCommand(teleopJoystickDriveCommand);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new SequentialCommandGroup(visionDrivePoint1, visionDrivePoint2, visionDrivePoint3);
  }
}