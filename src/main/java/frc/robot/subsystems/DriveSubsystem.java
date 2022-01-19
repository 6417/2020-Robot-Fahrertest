// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.DriveCommand;
import frc.robot.utilities.Controller;

public class DriveSubsystem extends SubsystemBase {
  
  private static DriveSubsystem instance;

  public static DifferentialDrive tank;

  private CANSparkMax motorL;
  private CANSparkMax motorL2;
  private CANSparkMax motorR;
  private CANSparkMax motorR2;

  private RelativeEncoder encoderL;
  private RelativeEncoder encoderR;

  private AHRS navx;
  private DifferentialDriveOdometry odometry;
  public DifferentialDriveKinematics kinematics;

  private double speed = Constants.Drive.normalSpeed;

  public static DriveSubsystem getInstance() {
    if (instance == null) {
      instance = new DriveSubsystem();
    }
    return instance;
  }

  /** Creates a new ExampleSubsystem. */
  private DriveSubsystem() {
    configMotors();
    tank = new DifferentialDrive(motorL, motorR);

    DriveSubsystem.instance = this;

    setDefaultCommand(new DriveCommand());
  }

  private void configMotors() {
      motorL = new CANSparkMax(Constants.Drive.motorL_ID, MotorType.kBrushless); 
      motorL2 = new CANSparkMax(Constants.Drive.motorL2_ID, MotorType.kBrushless); 
      motorR = new CANSparkMax(Constants.Drive.motorR_ID, MotorType.kBrushless); 
      motorR2 = new CANSparkMax(Constants.Drive.motorR2_ID, MotorType.kBrushless);

      motorL.restoreFactoryDefaults();
      motorL2.restoreFactoryDefaults();
      motorR.restoreFactoryDefaults();
      motorR2.restoreFactoryDefaults();

      motorL.setIdleMode(IdleMode.kBrake);
      motorL2.setIdleMode(IdleMode.kBrake);
      motorR.setIdleMode(IdleMode.kBrake);
      motorR2.setIdleMode(IdleMode.kBrake);

      motorL2.follow(motorL);
      motorR2.follow(motorR);

      motorL.setInverted(true);
      motorR.setInverted(true);

      encoderL = motorL.getEncoder();
      encoderR = motorR.getEncoder();

      navx = new AHRS(Port.kMXP);

      odometry = new DifferentialDriveOdometry(new Rotation2d(0), new Pose2d(new Translation2d(0, 0), new Rotation2d(0)));
      kinematics = new DifferentialDriveKinematics(Constants.Drive.trackWidthMeters);

      resetSensors();
  }

  public void resetSensors() {
    navx.reset();
    encoderL.setPosition(0);
    encoderR.setPosition(0);
  }

  public void drive() {
    tank.arcadeDrive(-Controller.getJoystick().getX()*this.speed, Controller.getJoystick().getY()*this.speed);
  }

  public void setSpeed(double maxSpeed) {
    this.speed = maxSpeed;
  }

  private double getLeftWheelDistance() {
    return encoderL.getPosition() / Constants.Drive.encoderToMetersConversion;
  }

  private double getRightWheelDistance() {
    return encoderR.getPosition() / -Constants.Drive.encoderToMetersConversion;
  }

  private void updateOdometry() {
    odometry.update(navx.getRotation2d(), getLeftWheelDistance(), getRightWheelDistance());
  }

  public Pose2d getPosition() {
    return odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(encoderL.getVelocity() / (60 * Constants.Drive.encoderToMetersConversion),
                                            encoderR.getVelocity() / (60 * Constants.Drive.encoderToMetersConversion));
  }

  public void resetOdometry(Pose2d setPoint) {
    odometry.resetPosition(setPoint, setPoint.getRotation());
  }

  public void stop() {
    tank.tankDrive(0, 0);
  }

  public void tankDriveVolts(double left, double right) {
    motorL.setVoltage(-left);
    motorR.setVoltage(right);
    tank.feed();
  }

  @Override
  public void periodic() {
    updateOdometry();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
      builder.addStringProperty("OdometryPoseX", () -> getPosition().toString(), null);
      builder.addStringProperty("LeftDist", () -> getWheelSpeeds().toString(), null);
      builder.addDoubleProperty("Angle", () -> navx.getAngle(), null);
      super.initSendable(builder);
  }
}
