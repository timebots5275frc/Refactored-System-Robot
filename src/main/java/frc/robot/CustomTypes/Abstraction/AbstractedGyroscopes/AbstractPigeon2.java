package frc.robot.CustomTypes.Abstraction.AbstractedGyroscopes;

import com.ctre.phoenix.sensors.Pigeon2;

import frc.robot.CustomTypes.Abstraction.Gyroscope;

public class AbstractPigeon2 implements Gyroscope
{
    Pigeon2 pigeon;

    public AbstractPigeon2(Pigeon2 p)
    {
        pigeon = p;
    }

    @Override
    public double getPitch() {
        return pigeon.getPitch();
    }

    @Override
    public double getYaw() {
        return pigeon.getYaw();
    }

    @Override
    public double getRoll() {
        return pigeon.getRoll();
    }

    @Override
    public void zeroYaw() {
       pigeon.setYaw(0);
    }
}