package frc.robot.subsystems.Vision;

import frc.robot.CustomTypes.Math.SillyMath;
import frc.robot.CustomTypes.Math.Vector2;

public class VisionDriveCalculator {
    static Vision vision;

    public static void SetVisionReference(Vision vision) { VisionDriveCalculator.vision = vision; }

    public static double rotateTowardsTarget()
    {
        final double rotSpeed = .5f;

        return -SillyMath.clamp(vision.HorizontalOffsetFromTarget() / 15, -1, 1) * rotSpeed;
    }

    public static Vector2 GetDirectionToAprilTagOffset(Vector2 aprilTagOffset)
    {
        Vector2 targetOffsetRelativeToRobot = new Vector2(vision.TargetPosInRobotSpace().x, vision.TargetPosInRobotSpace().z);

        return Vector2.zero;
    }
}
