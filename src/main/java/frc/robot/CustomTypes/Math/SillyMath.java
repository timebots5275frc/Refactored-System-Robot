package frc.robot.CustomTypes.Math;

public class SillyMath {
    public static double clamp(double in, double min, double max)
    {
        if (in > max) { return max; }
        else if (in < min) { return min; }

        return in;
    }
}
