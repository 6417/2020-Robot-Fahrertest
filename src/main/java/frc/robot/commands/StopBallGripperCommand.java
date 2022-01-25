package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BallsGripperSubsystem;

public class StopBallGripperCommand extends CommandBase {
    public StopBallGripperCommand() {
        addRequirements(BallsGripperSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        BallsGripperSubsystem.getInstance().closeGripper();
    }

    @Override
    public void execute() {
        BallsGripperSubsystem.getInstance().runGripper(0);
        BallsGripperSubsystem.getInstance().runTunnel(0);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
