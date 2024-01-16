package frc.robot.subsystems.Vision;

import frc.robot.CustomTypes.Math.Vector2;

public class VisionDriveCalculator {
    static Vision vision;

    public static void SetVisionReference(Vision vision) { VisionDriveCalculator.vision = vision; }

    public static double rotateTowardsTarget()
    {
        final double rotSpeed = .01f;

        if (vision.AprilTagID() != -1 && false)
        {
            return Math.signum(vision.HorizontalOffsetFromTarget()) * rotSpeed;
        }

        return 0;
    }
}
