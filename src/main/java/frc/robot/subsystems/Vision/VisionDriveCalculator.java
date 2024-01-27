package frc.robot.subsystems.Vision;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.VisionConstants;
import frc.robot.CustomTypes.Math.SillyMath;
import frc.robot.CustomTypes.Math.Vector2;

public class VisionDriveCalculator {
    static Vision vision;

    public static void SetVisionReference(Vision vision) { VisionDriveCalculator.vision = vision; }

    public static double rotateTowardsTarget()
    {
        return -SillyMath.clamp(vision.HorizontalOffsetFromAprilTag() / 22, -1, 1);
    }

    public static Vector2 GetVelocityToAprilTagOffset(Vector2 aprilTagOffset)
    {
        if (vision.hasValidData())
        {
            Vector2 aprilTagOffsetInRobotSpace = Vector2.rotate(aprilTagOffset, Math.toRadians(vision.AprilTagRotInRobotSpace().y + 180));
            Vector2 aprilTagInRobotSpace = new Vector2(vision.AprilTagPosInRobotSpace().x, vision.AprilTagPosInRobotSpace().z);
            Vector2 targetPositionInRobotSpace = aprilTagInRobotSpace.add(aprilTagOffsetInRobotSpace);

            SmartDashboard.putString("AT Offset in Robot Space", aprilTagOffsetInRobotSpace.toString(3));
            SmartDashboard.putString("AT in robot space", aprilTagInRobotSpace.toString(3));
            SmartDashboard.putString("Target Position in Robot Space", targetPositionInRobotSpace.toString(3));
            SmartDashboard.putNumber("Magnientuecd", targetPositionInRobotSpace.magnitude());

            return ApplyEasingToVector2(targetPositionInRobotSpace);
        }

        return Vector2.zero;
    }

    static Vector2 ApplyEasingToVector2(Vector2 direction)
    {
        double sqrtMagnitude = Math.sqrt(direction.magnitude() + 1) - 1;
        //double sqrtMagnitude = Math.pow(direction.magnitude() + 1, .9999) - 1;

        return direction.normalized().times(sqrtMagnitude);
    }
}
