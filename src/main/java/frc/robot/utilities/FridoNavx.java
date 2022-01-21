package frc.robot.utilities;

import com.kauailabs.navx.frc.AHRS;

public class FridoNavx extends AHRS{
    private double navxOffset = 0;

    public void setAngleOffset(double angle) {
        navxOffset = angle;
    }

    @Override
    public double getAngle() {
        return super.getAngle() + navxOffset;
    }
}
