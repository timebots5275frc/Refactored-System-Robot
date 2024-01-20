package frc.robot.subsystems.Vision;

import frc.robot.CustomTypes.Math.SillyMath;

public class VisionDriveCalculator {
    static Vision vision;

    public static void SetVisionReference(Vision vision) { VisionDriveCalculator.vision = vision; }

    public static double rotateTowardsTarget()
    {
        final double rotSpeed = .5f;

        return -SillyMath.clamp(vision.HorizontalOffsetFromTarget() / 15, -1, 1) * rotSpeed;
    }
}
