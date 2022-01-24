package frc.robot.utilities;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;
import frc.robot.commands.RecordTrajectoryCommand;
import frc.robot.commands.SpeedCommand;

public class Controller {
  
  private static Controller instance;

  private static Joystick inUseJoystick;
  private static JoystickButton slowButton;
  private static JoystickButton recordButton;

  public static Joystick getJoystick() {
    if (instance == null) {
      instance = new Controller();
    }
    return inUseJoystick;
  }

  /** Creates a new ExampleSubsystem. */
  private Controller() {
      inUseJoystick = new Joystick(Constants.Controller.inUseJoystick_ID);
      slowButton = new JoystickButton(inUseJoystick, Constants.Controller.slowButton_ID);
      recordButton = new JoystickButton(inUseJoystick, 2);

      slowButton.whileHeld(new SpeedCommand());
      recordButton.whileHeld(new RecordTrajectoryCommand());
  }
}
