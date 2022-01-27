package frc.robot;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utilities.PathviewerLoader;
import frc.robot.utilities.RamseteCommandGenerator;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  @Override
  public void robotInit() {
    DriveSubsystem.getInstance().resetSensors();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    String trajectoryJSON = "paths/output/TrajectoryRecording.wpilib.json";
    // String trajectoryJSON = "paths/output/GeradeAus4m.wpilib.json";

    Trajectory pathWeavertest = PathviewerLoader.loadTrajectory(trajectoryJSON);

    m_autonomousCommand = RamseteCommandGenerator.generateRamseteCommand(pathWeavertest);

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    DriveSubsystem.getInstance().resetSensors();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
