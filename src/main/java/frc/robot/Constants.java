package frc.robot;

public final class Constants {
    public static class Drive {
        public static final double slowSpeed = 0.375;
        public static final double normalSpeed = 1;
        public static final double encoderToMetersConversion = -22.14274406;

        public static final int motorL_ID = 10; // master
        public static final int motorL2_ID = 11; // follower
        public static final int motorR_ID = 12; // master
        public static final int motorR2_ID = 13; // follower
    }

    public static class Controller {
        public static final int inUseJoystick_ID = 0;
        public static final int slowButton_ID = 8;
    }
}
