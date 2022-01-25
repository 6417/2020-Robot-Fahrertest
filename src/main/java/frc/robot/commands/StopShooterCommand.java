package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BallsGripperSubsystem;
import frc.robot.subsystems.ShootBallSubsystem;

public class StopShooterCommand extends CommandBase{
    public StopShooterCommand() {
        addRequirements(ShootBallSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        ShootBallSubsystem.getInstance().setThrower(0);
        BallsGripperSubsystem.getInstance().runTunnel(0);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}