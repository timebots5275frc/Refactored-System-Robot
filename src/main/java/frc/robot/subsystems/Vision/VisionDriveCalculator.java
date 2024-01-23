package frc.robot.subsystems.Vision;

import frc.robot.CustomTypes.Math.SillyMath;
import frc.robot.CustomTypes.Math.Vector2;

public class VisionDriveCalculator {
    static Vision vision;

    public static void SetVisionReference(Vision vision) { VisionDriveCalculator.vision = vision; }

    public static double rotateTowardsTarget()
    {
        return -SillyMath.clamp(vision.HorizontalOffsetFromTarget() / 15, -1, 1);
    }

    public static Vector2 GetDirectionToAprilTagOffset(Vector2 aprilTagOffset)
    {
        if (vision.hasValidData())
        {
            Vector2 aprilTagOffsetInRobotSpace = Vector2.rotate(aprilTagOffset, Math.toRadians(vision.TargetRotInRobotSpace().y));
            Vector2 aprilTagInRobotSpace = new Vector2(vision.TargetPosInRobotSpace().x, vision.TargetPosInRobotSpace().z);
            Vector2 targetPositionInRobotSpace = aprilTagInRobotSpace.add(aprilTagOffsetInRobotSpace);

            return Vector2.clampMagnitude(targetPositionInRobotSpace, 1);
        }

        return Vector2.zero;
    }
}
