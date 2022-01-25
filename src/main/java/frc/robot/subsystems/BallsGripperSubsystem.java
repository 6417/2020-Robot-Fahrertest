package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class BallsGripperSubsystem extends SubsystemBase{
    private static BallsGripperSubsystem instance;

    private DoubleSolenoid platteolenoid;
    private DoubleSolenoid gripperSplenoid;

    private TalonSRX tunnel;
    private TalonSRX grippeTalonSRX;

    public static BallsGripperSubsystem getInstance() {
        if (instance == null) {
            instance = new BallsGripperSubsystem();
        }
        return instance;
    }

    private BallsGripperSubsystem() {
        platteolenoid = new DoubleSolenoid(30, PneumaticsModuleType.CTREPCM, 4, 5);
        gripperSplenoid = new DoubleSolenoid(30, PneumaticsModuleType.CTREPCM, 6, 7);

        extendPlatte();

        tunnel = new TalonSRX(Constants.Gripper.motorTunnel);
        grippeTalonSRX = new TalonSRX(Constants.Gripper.motorGripper);
    }

    public void extendGripper() {
        gripperSplenoid.set(Value.kForward);
    }

    public void extendPlatte() {
        platteolenoid.set(Value.kReverse);
    }

    public void closeGripper() {
        gripperSplenoid.set(Value.kReverse);
    }

    public void closePlatte() {
        platteolenoid.set(Value.kForward);
    }

    public void runTunnel(double speed) {
        tunnel.set(ControlMode.PercentOutput, speed);
    }

    public void runGripper(double speed) {
        grippeTalonSRX.set(ControlMode.PercentOutput, -speed);
    }
}
