// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveKinematicsConstraint;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.DriveCommand;
import frc.robot.utilities.Controller;
import frc.robot.utilities.FridoNavx;

public class DriveSubsystem extends SubsystemBase {
  
  private static DriveSubsystem instance;

  public static DifferentialDrive tank;

  private CANSparkMax motorL;
  private CANSparkMax motorL2;
  private CANSparkMax motorR;
  private CANSparkMax motorR2;

  private RelativeEncoder encoderL;
  private RelativeEncoder encoderR;

  private FridoNavx navx;
  private DifferentialDriveOdometry odometry;
  private DifferentialDriveKinematics kinematics;

  private double speed = Constants.Drive.normalSpeed;

  private PIDController rightVelocityController;
  private PIDController leftVelocityController;

  //Constrains
  private SimpleMotorFeedforward motorFeedforward;
  private DifferentialDriveVoltageConstraint voltageConstraint;
  private DifferentialDriveKinematicsConstraint kinematicsConstraint;
  private CentripetalAccelerationConstraint centripetalAccelerationConstraint;

  //config
  private TrajectoryConfig trajectoryConfig;

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
    
    navx = new FridoNavx(Port.kMXP);
    
    odometry = new DifferentialDriveOdometry(new Rotation2d(0), new Pose2d(new Translation2d(0, 0), new Rotation2d(0)));
    kinematics = new DifferentialDriveKinematics(Constants.Drive.trackWidthMeters);
    
    resetSensors();
    
    DriveSubsystem.instance = this;
    
    setDefaultCommand(new DriveCommand());
    
    configSimpleMotorFeedforward();

    configConstrains();
    
    configTrajectoryConfig();
  }
  
  public DifferentialDriveKinematics getDriveKinematics() {
    return kinematics;
  }

  public DifferentialDriveVoltageConstraint getVoltageConstrain() {
    return voltageConstraint;
  }

  public DifferentialDriveKinematicsConstraint getKinematicsConstrain() {
    return kinematicsConstraint;
  }

  public CentripetalAccelerationConstraint getCentripetConstraint() {
    return centripetalAccelerationConstraint;
  }

  public TrajectoryConfig getTrajectoryConfig() {
    return trajectoryConfig;
  }

  public PIDController getLeftVelocityController() {
    return leftVelocityController;
  }
  
  public PIDController getRightVelocityController() {
    return rightVelocityController;
  }

  private double getLeftWheelDistance() {
    return encoderL.getPosition() / Constants.Drive.encoderToMetersConversion;
  }
  
  private double getRightWheelDistance() {
    return encoderR.getPosition() / -Constants.Drive.encoderToMetersConversion;
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(encoderL.getVelocity() / (60 * Constants.Drive.encoderToMetersConversion),
                                           -encoderR.getVelocity() / (60 * Constants.Drive.encoderToMetersConversion));
  }
  
  public Pose2d getPosition() {
    return odometry.getPoseMeters();
  }

  public SimpleMotorFeedforward getMotorFeedforward() {
    return motorFeedforward;
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

      rightVelocityController = new PIDController(Constants.Drive.kP, Constants.Drive.kI, Constants.Drive.kD);
      leftVelocityController = new PIDController(Constants.Drive.kP, Constants.Drive.kI, Constants.Drive.kD);
  }

  private void configSimpleMotorFeedforward() {
      motorFeedforward = new SimpleMotorFeedforward(
        Constants.Drive.ksMeters,
        Constants.Drive.kvMetersPerSecoond,
        Constants.Drive.ka);
  }

  private void configConstrains() {
    configVoltageConstrain();
    configKinematicsConstrain();
    configCetripedalAccelerationConstrain();
  }

  private void configVoltageConstrain() {
    voltageConstraint = new DifferentialDriveVoltageConstraint(
        motorFeedforward,
        kinematics,
      10);
  }

  private void configKinematicsConstrain() {
    kinematicsConstraint = new DifferentialDriveKinematicsConstraint(
      kinematics,
      Constants.Drive.kMaxSpeed);
  }

  private void configCetripedalAccelerationConstrain() {
    centripetalAccelerationConstraint = new CentripetalAccelerationConstraint(
      Constants.Drive.kMaxCentripetalAcceleration);
  }

  private void configTrajectoryConfig() {
    trajectoryConfig = new TrajectoryConfig(
      Constants.Drive.kMaxSpeed, 
      Constants.Drive.kMaxAcceleration).setKinematics(kinematics).addConstraint(voltageConstraint).addConstraint(kinematicsConstraint).addConstraint(centripetalAccelerationConstraint);
  }

  public void resetSensors() {
    navx.reset();
    odometry.resetPosition(new Pose2d(0, 0, new Rotation2d(0)), new Rotation2d(0));
    encoderL.setPosition(0);
    encoderR.setPosition(0);
  }

  public void resetOdometry(Pose2d setPoint) {
    odometry.resetPosition(setPoint, setPoint.getRotation());
    navx.setAngleOffset(setPoint.getRotation().getDegrees());
  }

  private void updateOdometry() {
    odometry.update(Rotation2d.fromDegrees(navx.getAngle()), getLeftWheelDistance(), getRightWheelDistance());
  }

  public void drive() {
    tank.arcadeDrive(-Controller.getJoystick().getX()*this.speed, Controller.getJoystick().getY()*this.speed);
  }

  public void setSpeed(double maxSpeed) {
    this.speed = maxSpeed;
  }

  public void stop() {
    tank.stopMotor();
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
      builder.addDoubleProperty("ANGLE_ODOMETRY", () -> getPosition().getRotation().getDegrees(),null);
      builder.addDoubleProperty("rightSpeed", () -> getWheelSpeeds().rightMetersPerSecond, null);
      builder.addDoubleProperty("leftSpeed", () -> getWheelSpeeds().leftMetersPerSecond, null);
      super.initSendable(builder);
  }
}
