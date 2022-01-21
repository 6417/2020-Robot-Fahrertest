package frc.robot.utilities;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class RamseteCommandGenerator {
    public static Command generateRamseteCommand(Trajectory path) {
        // Resetting all the sensors and the odometry on the robot and setting the initial pose
        DriveSubsystem.getInstance().resetSensors();
        DriveSubsystem.getInstance().resetOdometry(path.getInitialPose());

        // Generating the ramsete command
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

        // Finishing and returning the command
        return ramseteCommand.andThen(() -> DriveSubsystem.getInstance().stop());
    }
}