package frc.robot.utilities;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;

public class FridoNavx extends AHRS{
    private double navxOffset = 0;

    public FridoNavx(SPI.Port port) {
        super(port);
        super.calibrate();
        navxOffset = 0;
    }

    public void setAngleOffset(double angle) {
        navxOffset = angle;
    }

    @Override
    public void reset() {
        super.reset();
        navxOffset = 0;
    }

    // Overriding the getAngle to implement offsets
    @Override
    public double getAngle() {
        return -(super.getAngle() + navxOffset);
    }
}
