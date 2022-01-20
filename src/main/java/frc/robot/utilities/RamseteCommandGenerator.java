package frc.robot.utilities;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class RamseteCommandGenerator {
    public static Command generateRamseteCommand(Trajectory path) {
        DriveSubsystem.getInstance().resetSensors();
        DriveSubsystem.getInstance().resetOdometry(path.getInitialPose());

        RamseteCommand ramseteCommand = new RamseteCommand(
        path, 
        DriveSubsystem.getInstance()::getPosition, 
        new RamseteController(Constants.Drive.kRamseteB, Constants.Drive.kRamseteZeta), 
        DriveSubsystem.getInstance().getMotorFeedforward(),
        DriveSubsystem.getInstance().getDriveKinematics(), 
        DriveSubsystem.getInstance()::getWheelSpeeds, 
        DriveSubsystem.getInstance().getLeftVelocityController(),
        DriveSubsystem.getInstance().getRightVelocityController(),
        (leftSpeed, rightSpeed) -> {
            DriveSubsystem.getInstance().tankDriveVolts(leftSpeed, rightSpeed);
        },
        DriveSubsystem.getInstance());
        return ramseteCommand.andThen(() -> DriveSubsystem.getInstance().stop());
    }
}
