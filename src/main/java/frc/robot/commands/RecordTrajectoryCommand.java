// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utilities.TrajectoryCreator;

/** An example command that uses an example subsystem. */
public class RecordTrajectoryCommand extends CommandBase {
  TrajectoryCreator logger;
  Timer timer;
  double cooldown = 0.05;
  double previousTime;

  public RecordTrajectoryCommand() {
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    logger = new TrajectoryCreator("tmp/TrajectoryRecording.wpilib.json");
    DriveSubsystem.getInstance().resetSensors();
    timer = new Timer();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    cooldown -= (timer.get() - previousTime);
    previousTime = timer.get();
    if (cooldown <= 0)
    {
      logger.addDatapoint(DriveSubsystem.getInstance().getChassisSpeeds(), DriveSubsystem.getInstance().getPosition());
      cooldown = 0.05;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    logger.close();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
