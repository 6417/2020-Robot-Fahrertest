package frc.robot.utilities;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;
import frc.robot.commands.RecordTrajectoryCommand;
import frc.robot.commands.RunBallGripperCommand;
import frc.robot.commands.SpeedCommand;
import frc.robot.commands.StopBallGripperCommand;
import frc.robot.commands.RunShooterCommand;
import frc.robot.commands.ShootCommandGrop;

public class Controller {
  
  private static Controller instance;

  private static Joystick inUseJoystick;
  private static JoystickButton slowButton;
  private static JoystickButton recordButton;

  private static JoystickButton gripperOpenButton;
  private static JoystickButton gripperCloseButton;

  private static JoystickButton shooter;

  public static Joystick getJoystick() {
    if (instance == null) {
      instance = new Controller();
    }
    return inUseJoystick;
  }

  private static Runnable runCommandAndCancelWhenPressedAgain(CommandBase command) {
    return () -> {
      if (CommandScheduler.getInstance().isScheduled(command))
        CommandScheduler.getInstance().cancel(command);
      else
        CommandScheduler.getInstance().schedule(command);
    };
  }

  /** Creates a new ExampleSubsystem. */
  private Controller() {
      inUseJoystick = new Joystick(Constants.Controller.inUseJoystick_ID);
      slowButton = new JoystickButton(inUseJoystick, Constants.Controller.slowButton_ID);
      recordButton = new JoystickButton(inUseJoystick, 2);

      gripperCloseButton = new JoystickButton(inUseJoystick, 5);
      gripperOpenButton = new JoystickButton(inUseJoystick, 3);

      shooter = new JoystickButton(inUseJoystick, 1);

      slowButton.whileHeld(new SpeedCommand());
      recordButton.whileHeld(new RecordTrajectoryCommand());

      gripperOpenButton.whenPressed(runCommandAndCancelWhenPressedAgain(new RunBallGripperCommand()));
      gripperCloseButton.whenPressed(runCommandAndCancelWhenPressedAgain(new StopBallGripperCommand()));

      shooter.whileHeld(new ShootCommandGrop());
  }
}
