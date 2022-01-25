package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BallsGripperSubsystem;
import frc.robot.subsystems.ShootBallSubsystem;

public class ThrowerCommand extends CommandBase{
    public ThrowerCommand() {
        addRequirements(ShootBallSubsystem.getInstance());
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        ShootBallSubsystem.getInstance().setThrower(0.7);
        BallsGripperSubsystem.getInstance().runTunnel(0.7);
    }

    @Override
    public void end(boolean interrupted) {
        ShootBallSubsystem.getInstance().setThrower(0);
        BallsGripperSubsystem.getInstance().runTunnel(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}