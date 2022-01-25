package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShootBallSubsystem;

public class RunShooterCommand extends CommandBase{
    public RunShooterCommand() {
        addRequirements(ShootBallSubsystem.getInstance());
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        ShootBallSubsystem.getInstance().setUpper(1);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}