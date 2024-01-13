package frc.robot.subsystems.Vision;

import frc.robot.CustomTypes.Math.Vector2;

public class VisionDriveCalculator {
    static Vision vision;

    public static void SetVisionReference(Vision vision) { VisionDriveCalculator.vision = vision; }

    public static double rotateTowardsTarget()
    {
        final double rotSpeed = .1f;

        if (vision.aprilTagID != -1)
        {
            return Math.signum(vision.horizontalOffsetFromTarget) * rotSpeed;
        }

        return 0;
    }
}
