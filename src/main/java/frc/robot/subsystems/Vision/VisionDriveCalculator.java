package frc.robot.subsystems.Vision;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.CustomTypes.Math.SillyMath;
import frc.robot.CustomTypes.Math.Vector2;

public class VisionDriveCalculator {
    static Vision vision;

    public static void SetVisionReference(Vision vision) { VisionDriveCalculator.vision = vision; }

    public static double rotateTowardsTarget()
    {
        return -SillyMath.clamp(vision.HorizontalOffsetFromAprilTag() / 22, -1, 1);
    }

    public static Vector2 GetDirectionToAprilTagOffset(Vector2 aprilTagOffset)
    {
        if (vision.hasValidData())
        {
            Vector2 aprilTagOffsetInRobotSpace = Vector2.rotate(aprilTagOffset, Math.toRadians(vision.AprilTagRotInRobotSpace().y + 90));
            Vector2 aprilTagInRobotSpace = new Vector2(vision.AprilTagPosInRobotSpace().x, vision.AprilTagPosInRobotSpace().z);
            Vector2 targetPositionInRobotSpace = aprilTagInRobotSpace.add(aprilTagOffsetInRobotSpace);

            SmartDashboard.putString("AT Offset in Robot Space", aprilTagOffsetInRobotSpace.toString(3));
            SmartDashboard.putString("Target Position in Robot Space", targetPositionInRobotSpace.toString(3));

            return Vector2.clampMagnitude(targetPositionInRobotSpace, 1);
        }

        return Vector2.zero;
    }
}
