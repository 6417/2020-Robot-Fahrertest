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

    private DoubleSolenoid platteSolenoid;
    private DoubleSolenoid gripperSolenoid;

    private TalonSRX tunnel;
    private TalonSRX grippeTalonSRX;

    public static BallsGripperSubsystem getInstance() {
        if (instance == null) {
            instance = new BallsGripperSubsystem();
        }
        return instance;
    }

    private BallsGripperSubsystem() {
        platteSolenoid = new DoubleSolenoid(Constants.Gripper.compressor_ID, PneumaticsModuleType.CTREPCM, Constants.Gripper.underPlattePneumatic_ID, Constants.Gripper.upperPlattePneumatic_ID);
        gripperSolenoid = new DoubleSolenoid(Constants.Gripper.compressor_ID, PneumaticsModuleType.CTREPCM, Constants.Gripper.underGrpperPneumatic_ID, Constants.Gripper.upperGrpperPneumatic_ID);

        extendPlatte();

        tunnel = new TalonSRX(Constants.Gripper.motorTunnel_ID);
        grippeTalonSRX = new TalonSRX(Constants.Gripper.motorGripper_ID);
    }

    public void extendGripper() {
        gripperSolenoid.set(Value.kForward);
    }

    public void extendPlatte() {
        platteSolenoid.set(Value.kReverse);
    }

    public void closeGripper() {
        gripperSolenoid.set(Value.kReverse);
    }

    public void closePlatte() {
        platteSolenoid.set(Value.kForward);
    }

    public void runTunnel(double speed) {
        tunnel.set(ControlMode.PercentOutput, speed);
    }

    public void runGripper(double speed) {
        grippeTalonSRX.set(ControlMode.PercentOutput, -speed);
    }
}
