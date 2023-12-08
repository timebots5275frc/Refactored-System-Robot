package frc.robot.CustomTypes.Abstraction.AbstractedGyroscopes;

import com.ctre.phoenix.sensors.PigeonIMU;

import frc.robot.CustomTypes.Abstraction.Gyroscope;

public class AbstractPigeonIMU implements Gyroscope
{
    PigeonIMU pigeon;

    public AbstractPigeonIMU(PigeonIMU p)
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