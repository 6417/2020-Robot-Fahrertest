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

        public static final double ksMeters = 0.12;
        public static final double kvMetersPerSecoond = 2.89;
        public static final double ka = 0.45;

        public static final double kMaxSpeed = 1.5;
        public static final double kMaxAcceleration = 0.7;
        public static final double kMaxCentripetalAcceleration = 0.5;

        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;

        public static final double kP = 0.035271;
        public static final double kI = 0;
        public static final double kD = 0;
    }

    public static class Controller {
        public static final int inUseJoystick_ID = 0;
        public static final int slowButton_ID = 8;
        public static final int recordButton_ID = 2;
        public static final int gripperCloseButton_ID = 5;
        public static final int gripperOpenButton_ID = 3;
        public static final int shooterButton_ID = 1;
    }

    public static class Gripper {
        public static final int motorTunnel_ID = 15;
        public static final int motorGripper_ID = 14;
        public static final int underPlattePneumatic_ID = 4;
        public static final int upperPlattePneumatic_ID = 5;
        public static final int underGrpperPneumatic_ID = 6;
        public static final int upperGrpperPneumatic_ID = 7;
        public static final int compressor_ID = 30;
    }

    public static class Thrower {
        public static final int leftUpperMotor_ID = 23;
        public static final int rightUpperMotor_ID = 22;
        public static final int underMotor_ID = 21;
    }
}
