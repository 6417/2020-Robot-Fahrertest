package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BallsGripperSubsystem;

public class RunBallGripperCommand extends CommandBase {
    public RunBallGripperCommand() {
        addRequirements(BallsGripperSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        BallsGripperSubsystem.getInstance().extendGripper();
    }

    @Override
    public void execute() {
        BallsGripperSubsystem.getInstance().runGripper(0.7);
        BallsGripperSubsystem.getInstance().runTunnel(0.7);
    }

    @Override
    public void end(boolean interrupted) {
        BallsGripperSubsystem.getInstance().runTunnel(0);
        BallsGripperSubsystem.getInstance().runGripper(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
