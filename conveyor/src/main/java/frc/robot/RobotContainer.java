package frc.robot;

import static edu.wpi.first.wpilibj.XboxController.Button;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.FeederSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final FeederSubsystem m_feeder = new FeederSubsystem();

  // The autonomous routines     
  private final Command m_feedCommand = 
        new FunctionalCommand(
            // init : do nothing
            () -> {}, 
            // execute : start feeding the ball
            () -> m_feeder.feed(Constants.maxFeederMotorSpeed), 
            // when interrupted : stop feeding
            interrupt -> m_feeder.stop(), 
            // isFinished?  : done when a ball is present
            () -> m_feeder.isBallPresent(), 
            // requirements
            m_feeder);

  // The driver's controller
  XboxController m_driverController = new XboxController(Constants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
       // Configure default commands
    // Set the default drive command to split-stick arcade drive
    m_feeder.setDefaultCommand(
        // A split-stick arcade command, with forward/backward controlled by the left
        // hand, and turning controlled by the right.
        new RunCommand(
            () -> m_feeder.feed(m_driverController.getY(GenericHID.Hand.kLeft)),
            m_feeder));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link PS4Controller}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    new JoystickButton(m_driverController, Button.kBumperLeft.value)
        .whenPressed(m_feedCommand);

    new JoystickButton(m_driverController, Button.kBumperRight.value)
        .whenPressed(m_feedCommand::cancel);

    new JoystickButton(m_driverController, Button.kA.value)
        .whenPressed(() -> m_feeder.feed(Constants.maxFeederMotorSpeed))
        .whenReleased(() -> m_feeder.stop());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_feedCommand;
  }
}