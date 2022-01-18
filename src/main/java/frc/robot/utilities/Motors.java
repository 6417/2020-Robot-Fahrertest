package frc.robot.utilities;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Motors {

    public static Drive drive;
    private static Motors instance;

    public static Motors getInstance() {
        if (instance == null) {
            instance = new Motors();
        }
        return instance;
    }
   
    public class Drive {
        public CANSparkMax motorL;
        private CANSparkMax motorL2;
        public CANSparkMax motorR;
        private CANSparkMax motorR2;

        public void configMotors() {
            motorL = new CANSparkMax(10, MotorType.kBrushless); 
            motorL2 = new CANSparkMax(11, MotorType.kBrushless); 
            motorR = new CANSparkMax(12, MotorType.kBrushless); 
            motorR2 = new CANSparkMax(13, MotorType.kBrushless);

            motorL.restoreFactoryDefaults();
            motorL2.restoreFactoryDefaults();
            motorR.restoreFactoryDefaults();
            motorR2.restoreFactoryDefaults();

            motorL2.follow(motorL);
            motorR2.follow(motorR);

            motorL.setInverted(true);
            motorR.setInverted(true);
        }
    }

}
