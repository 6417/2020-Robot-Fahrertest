package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShootBallSubsystem extends SubsystemBase{
    private static ShootBallSubsystem instance;

    private CANSparkMax leftUpperMotor;
    private CANSparkMax rightUpperMotor;
    private CANSparkMax underMotor;

    private MotorControllerGroup upperMotor;

    public static ShootBallSubsystem getInstance() {
        if (instance == null) {
            instance = new ShootBallSubsystem();
        }
        return instance;
    }

    private ShootBallSubsystem() {
        leftUpperMotor = new CANSparkMax(Constants.Thrower.leftUpperMotor_ID, MotorType.kBrushless);
        rightUpperMotor = new CANSparkMax(Constants.Thrower.rightUpperMotor_ID, MotorType.kBrushless);
        underMotor = new CANSparkMax(Constants.Thrower.underMotor_ID, MotorType.kBrushless);

        leftUpperMotor.restoreFactoryDefaults();
        rightUpperMotor.restoreFactoryDefaults();
        underMotor.restoreFactoryDefaults();

        rightUpperMotor.setInverted(true);
        underMotor.setInverted(true);

        upperMotor = new MotorControllerGroup(leftUpperMotor, rightUpperMotor);
    }

   public void setThrower(double speed) {
       upperMotor.set(speed);
       underMotor.set(speed);
   }

   public void setUpper(double speed) {
       upperMotor.set(speed);
   }
}
