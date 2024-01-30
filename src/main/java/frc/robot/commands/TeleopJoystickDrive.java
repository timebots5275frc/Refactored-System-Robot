// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Vector;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain.SwerveDrive;
import frc.robot.subsystems.Vision.VisionFollowAprilTag;
import frc.robot.CustomTypes.Math.Vector2;

public class TeleopJoystickDrive extends Command {

    private SwerveDrive drivetrain;
    private VisionFollowAprilTag vision_followAprilTag;

    private Joystick driveStick;
    private boolean fieldRelative;

    private static final Vector2[] targetPositions = new Vector2[] {
        new Vector2(0, 1),
        new Vector2(.5, 1),
        new Vector2(-.5, 1),
        new Vector2(0, 2)
    };

    int currentTargetPos = -1;

    /**
     * Creates a new DefaultDrive.
     *
     * @param subsystem The drive subsystem this command wil run on.
     * @param joystick  The control input for driving
     */
    public TeleopJoystickDrive(SwerveDrive _swerveDrive, Joystick _driveStick, boolean _fieldRelative) {
        this.drivetrain = _swerveDrive;
        this.driveStick = _driveStick;
        this.fieldRelative = _fieldRelative;
        this.vision_followAprilTag = new VisionFollowAprilTag(_swerveDrive);

        addRequirements(_swerveDrive);
    }

    public void SetFieldRelative(boolean setboolfieldRelative) {
        System.out.println("SetFieldRelative = " + setboolfieldRelative);
        this.fieldRelative = setboolfieldRelative;
    }

    @Override
    public void initialize() {
        drivetrain.resetPigeon();
    }

    @Override
    public void execute() {

        double xSpeed = this.smartJoystick(driveStick.getY() * -1, Constants.ControllerConstants.DEADZONE_DRIVE) * Constants.DriveConstants.MAX_DRIVE_SPEED;
        double ySpeed = this.smartJoystick(driveStick.getX() * -1, Constants.ControllerConstants.DEADZONE_DRIVE) * Constants.DriveConstants.MAX_DRIVE_SPEED;
        double rotRate = this.smartJoystick(driveStick.getTwist() * -1, Constants.ControllerConstants.DEADZONE_STEER) * Constants.DriveConstants.MAX_TWIST_RATE;

        double throttle = (-driveStick.getThrottle() + 1) / 2; // between 0 and 1 = 0% and 100%

        xSpeed *= throttle;
        ySpeed *= throttle;
        rotRate *= throttle;
        
        SmartDashboard.putNumber("Throttle teleJoy", throttle);
        SmartDashboard.putNumber("xSpeed teleJoy smart", xSpeed);
        SmartDashboard.putNumber("ySpeed teleJoy smart ", ySpeed);
        SmartDashboard.putNumber("rotRate teleJoy smart ", rotRate);

        if (driveStick.getRawButtonPressed(9)) { currentTargetPos = 0; }
        if (driveStick.getRawButtonPressed(10)) { currentTargetPos = 1; }
        if (driveStick.getRawButtonPressed(11)) { currentTargetPos = 2; }
        if (driveStick.getRawButtonPressed(12)) { currentTargetPos = 3; }

        if (ySpeed != 0 || xSpeed != 0) { currentTargetPos = -1; }

        if (currentTargetPos != -1) { vision_followAprilTag.followWithOffset(targetPositions[currentTargetPos]); }
        else { drivetrain.drive(xSpeed, ySpeed, rotRate, fieldRelative); }
    }

    /**
     * 
     * @param _in
     * @param deadZoneSize between -1 and 1
     * @return
     */
    public double smartJoystick(double _in, double deadZoneSize) {
        if (Math.abs(_in) < deadZoneSize) {
            return 0;
        }

        if (_in > 0) {
            return (_in - deadZoneSize) / (1 - deadZoneSize);
        } else if (_in < 0) {
            return (_in + deadZoneSize) / (1 - deadZoneSize);
        }
        return 0;
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    public void setFieldRelative(boolean bool) {
        this.fieldRelative = bool;
    }
}
