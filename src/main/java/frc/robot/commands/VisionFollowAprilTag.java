// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CustomTypes.Math.Vector2;
import frc.robot.subsystems.DriveTrain.SwerveDrive;
import frc.robot.subsystems.Vision.VisionDriveCalculator;

public class VisionFollowAprilTag extends Command {
  
  SwerveDrive swerveDrive;

  static Vector2 FollowOffset = new Vector2(.5, 1);

  public VisionFollowAprilTag(SwerveDrive swerveDrive) {
    this.swerveDrive = swerveDrive;

    addRequirements(swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    double turnSpeed = 1.2;
    double driveSpeed = 2;

    double rotationVelocity = VisionDriveCalculator.rotateTowardsTarget() * turnSpeed;
    Vector2 moveVelocity = VisionDriveCalculator.GetVelocityToAprilTagOffset(FollowOffset).times(driveSpeed);

    SmartDashboard.putString("AT Rotation velocity", rotationVelocity + "");
    SmartDashboard.putString("AT Move velocity", moveVelocity.toString(3));

    //if (moveVelocity.magnitude() < .1) { moveVelocity = Vector2.zero; }
    swerveDrive.drive(moveVelocity.y, moveVelocity.x, rotationVelocity, false);
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
