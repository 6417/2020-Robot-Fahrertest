// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utilities.RamseteCommandGenerator;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  // public Command getAutonomousCommand() {
  //   Trajectory testTrajectory = TrajectoryGenerator.generateTrajectory(
  //     new Pose2d(0, 0, Rotation2d.fromDegrees(0)), 
  //     List.of(
  //       new Translation2d(4, 0),
  //       new Translation2d(2, -2),
  //       new Translation2d(2, 2)
  //     ),
  //     new Pose2d(0, 0, Rotation2d.fromDegrees(180)),
  //     DriveSubsystem.getInstance().getTrajectoryConfig());
    
  //   //Autonous
  //   DriveSubsystem.getInstance().resetSensors();
  //   DriveSubsystem.getInstance().resetOdometry(testTrajectory.getInitialPose());

  //   RamseteCommand ramseteCommand = new RamseteCommand(
  //     testTrajectory, 
  //     DriveSubsystem.getInstance()::getPosition, 
  //     new RamseteController(Constants.Drive.kRamseteB, Constants.Drive.kRamseteZeta), 
  //     new SimpleMotorFeedforward(
  //       Constants.Drive.ksMeters,
  //       Constants.Drive.kvMetersPerSecoond,
  //       Constants.Drive.ka),
  //     DriveSubsystem.getInstance().getDriveKinematics(), 
  //     DriveSubsystem.getInstance()::getWheelSpeeds, 
  //     DriveSubsystem.getInstance().getLeftVelocityController(),
  //     DriveSubsystem.getInstance().getRightVelocityController(),
  //     (leftSpeed, rightSpeed) -> {
  //       DriveSubsystem.getInstance().tankDriveVolts(leftSpeed, rightSpeed);
  //     },
  //     DriveSubsystem.getInstance());
  //   return ramseteCommand.andThen(() -> DriveSubsystem.getInstance().stop());
  // }

  Trajectory testTrajectory = TrajectoryGenerator.generateTrajectory(
    new Pose2d(0, 0, Rotation2d.fromDegrees(0)), 
    List.of(
      new Translation2d(4, 0),
      new Translation2d(2, -2),
      new Translation2d(2, 2)
    ),
    new Pose2d(0, 0, Rotation2d.fromDegrees(180)),
    DriveSubsystem.getInstance().getTrajectoryConfig());

  Trajectory testTrajectory2 = TrajectoryGenerator.generateTrajectory(
    new Pose2d(0, 0, Rotation2d.fromDegrees(0)), 
    List.of(
      new Translation2d(2, 0),
      new Translation2d(1, -1),
      new Translation2d(2, 2)
    ),
    new Pose2d(1, 0, Rotation2d.fromDegrees(180)),
    DriveSubsystem.getInstance().getTrajectoryConfig());

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = RamseteCommandGenerator.generateRamseteCommand(testTrajectory2);

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
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
