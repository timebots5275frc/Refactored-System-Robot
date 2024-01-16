package frc.robot.CustomTypes;

public class SwerveModuleLocations {
    // IN METERS
    public final double LEFT_FRONT_WHEEL_X;
    public final double LEFT_FRONT_WHEEL_Y;
    public final double RIGHT_FRONT_WHEEL_X;
    public final double RIGHT_FRONT_WHEEL_Y;
    public final double RIGHT_REAR_WHEEL_X;
    public final double RIGHT_REAR_WHEEL_Y;
    public final double LEFT_REAR_WHEEL_X;
    public final double LEFT_REAR_WHEEL_Y;

    public SwerveModuleLocations(double LFWX, double LFWY, double RFWX, double RFWY, double RRWX, double RRWY, double LRWX, double LRWY)
    {
        LEFT_FRONT_WHEEL_X = LFWX;
        LEFT_FRONT_WHEEL_Y = LFWY;
        RIGHT_FRONT_WHEEL_X = RFWX;
        RIGHT_FRONT_WHEEL_Y = RFWY;
        RIGHT_REAR_WHEEL_X = RRWX;
        RIGHT_REAR_WHEEL_Y = RRWY;
        LEFT_REAR_WHEEL_X = LRWX;
        LEFT_REAR_WHEEL_Y = LRWY;
    }
}
