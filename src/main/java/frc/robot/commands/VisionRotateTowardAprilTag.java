// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain.SwerveDrive;
import frc.robot.subsystems.Vision.VisionDriveCalculator;

public class VisionRotateTowardAprilTag extends Command {

  SwerveDrive swerveDrive;
  Joystick driveStick;

  /** Creates a new VisionRotateTowardAprilTag. */
  public VisionRotateTowardAprilTag(SwerveDrive swerveDrive, Joystick driveStick) {
    this.swerveDrive = swerveDrive;
    this.driveStick = driveStick;

    addRequirements(swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerveDrive.drive(0, 0, VisionDriveCalculator.rotateTowardsTarget(), false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
