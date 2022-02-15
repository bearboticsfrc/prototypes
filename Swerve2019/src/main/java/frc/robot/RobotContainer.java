// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.DriveConstants.PivotPoint;
import frc.robot.commands.AutoRotate;
import frc.robot.commands.TargetDrive;
import frc.robot.subsystems.BlinkinSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive;
  private final LimelightSubsystem m_limeLight;
  private final BlinkinSubsystem m_blinkin;

  // The driver's controller
  private final XboxController m_driverController;

  // Autonomous Routines
  private final TargetDrive m_targetDrive;
  private final AutoRotate m_autoRotate;

  // A chooser for autonomous commands
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  // a hook to call aback to Robot to see if we are in Autonomous mode (for setting LEDs)
  private BooleanSupplier m_autoModeSupplier = (() -> { return false; });

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_blinkin = new BlinkinSubsystem(LEDConstants.kBlinkinPWMPort);
    m_robotDrive = new DriveSubsystem(m_blinkin);
    m_limeLight = new LimelightSubsystem();
    m_driverController = new XboxController(OIConstants.kDriverControllerPort);
    m_targetDrive = new TargetDrive(m_robotDrive, m_limeLight, m_blinkin, m_driverController::getLeftY, m_driverController::getLeftX);
    m_autoRotate = new AutoRotate(m_robotDrive);

    // Configure the button bindings
    configureButtonBindings();
    configureTriggers();
    configureAutonomousChooser();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), 0.1),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), 0.1),
                -MathUtil.applyDeadband(m_driverController.getRightX(), 0.1),
                PivotPoint.getByPOV(m_driverController.getPOV())),
            m_robotDrive));
    ShuffleboardTab tab = Shuffleboard.getTab("Drive System");
    tab.addBoolean("Target Mode", m_targetDrive::isScheduled);
    tab.addBoolean("Target Lock", m_limeLight::valid);
    m_limeLight.disableLED();
  }

  private void configureAutonomousChooser() {
    // Add commands to the autonomous command chooser
   // m_chooser.addOption("Simple Path", AutonomousCommandHelper.getSimplAutonomousCommand(m_robotDrive));
    m_chooser.setDefaultOption("Path Planner", AutonomousCommandHelper.getPathPlannerCommand(m_robotDrive));
    m_chooser.addOption("Auto Rotate", m_autoRotate);

    // Put the chooser on the dashboard
    SmartDashboard.putData(m_chooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {

    new JoystickButton(m_driverController, Button.kA.value)
        .whenPressed(() -> m_robotDrive.zeroHeading());

    new JoystickButton(m_driverController, Button.kB.value)
        .whenPressed(() -> m_limeLight.toggleLEDs());

    new JoystickButton(m_driverController, Button.kX.value)
        .toggleWhenPressed(m_targetDrive);

    new JoystickButton(m_driverController, Button.kRightBumper.value)
        .whenPressed(() -> m_robotDrive.setTurboMode(true))
        .whenReleased(() -> m_robotDrive.setTurboMode(false));

    new JoystickButton(m_driverController, Button.kLeftBumper.value)
        .whenPressed(() -> m_robotDrive.setFieldRelative(false))
        .whenReleased(() -> m_robotDrive.setFieldRelative(true));

  }

  private void configureTriggers() {

    // new Trigger(m_targetDrive::isScheduled)
    //     .whenActive(() -> m_blinkin.set(BlinkinSubsystem.Color.YELLOW))
    //     .whenInactive(() -> m_blinkin.set(BlinkinSubsystem.Color.BLUE));
    // new Trigger(m_robotDrive::getTurboMode)
    //     .whenActive(() -> m_blinkin.set(BlinkinSubsystem.Color.RED));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }


  public void setAutoModeSupplier(BooleanSupplier autoModeSupplier) {
    m_autoModeSupplier = autoModeSupplier;
  }

  public BlinkinSubsystem getBlinkin() {
    return m_blinkin;
  }

  public void autonomousInit() {
    m_blinkin.set(BlinkinSubsystem.Color.STROBE_BLUE);
  }

  public void teleopInit() {
    m_blinkin.set(BlinkinSubsystem.Color.BLUE);
  }

  public void disabledInit() {
    m_blinkin.set(BlinkinSubsystem.Color.BLUE_VIOLET);
  }

  public void periodic() {
    if (m_targetDrive.isScheduled() && !m_limeLight.valid()) {
      //m_driverController.setRumble(RumbleType.kLeftRumble, 0.25);
    } else {
      //m_driverController.setRumble(RumbleType.kLeftRumble, 0.0);
    }

    //if (!m_robotDrive.getFieldRelative()) {
    //  m_blinkin.set(BlinkinSubsystem.Color.HOT_PINK);
    //} else {
    //  if (m_autoModeSupplier.getAsBoolean()) {
        // Autonomous Mode
    //    m_blinkin.set(BlinkinSubsystem.Color.STROBE_BLUE);
    //  } else {
        //if (m_targetDrive.isScheduled()) {
        //  m_blinkin.set(BlinkinSubsystem.Color.YELLOW);
        //} else {
         // m_blinkin.set((BlinkinSubsystem.Color.BLUE));
    //    }
    //  }
    //}
  }

}
