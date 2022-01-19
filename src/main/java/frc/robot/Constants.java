package frc.robot;

public final class Constants {
    public static class Drive {
        public static final double slowSpeed = 0.375;
        public static final double normalSpeed = 1;
        public static final double encoderToMetersConversion = -22.14274406;

        public static final int motorL_ID = 10;
        public static final int motorL2_ID = 11;
        public static final int motorR_ID = 12;
        public static final int motorR2_ID = 13;

        public static final double trackWidthMeters = 0.5;

        public static final double ks = 0.087112;
        public static final double kv = 2.986;
        public static final double ka = 0.318;

        public static final double kMaxSpeed = 1;
        public static final double kMaxAcceleration = 0.8;

        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;

        // DEBUG KP VALUE IT IS WRONG
        public static final double kP = 1.385*Math.pow(10, -5);
        public static final double kI = 0;
        public static final double kD = 0;
    }

    public static class Controller {
        public static final int inUseJoystick_ID = 0;
        public static final int slowButton_ID = 8;
    }
}
