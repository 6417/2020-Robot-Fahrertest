package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.BallsGripperSubsystem;
import frc.robot.subsystems.ShootBallSubsystem;

public class ShootCommandGrop extends SequentialCommandGroup{
   public ShootCommandGrop() {
       addCommands(new RunShooterCommand(), new TimerCommand(1), new ThrowerCommand());
   } 

   @Override
   public void end(boolean interrupted) {
       ShootBallSubsystem.getInstance().setThrower(0);
       BallsGripperSubsystem.getInstance().runTunnel(0);
   }
}
