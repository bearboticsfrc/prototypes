// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AutoRotate;
import frc.robot.commands.TargetDrive;
import frc.robot.subsystems.BlinkinSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.List;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  // private final nosubsystem m_robotDrive = new nosubsystem();
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();

  private final LimelightSubsystem m_limeLight = new LimelightSubsystem();

  private BlinkinSubsystem m_blinkin = null; 

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  private final TargetDrive m_targetDrive = new TargetDrive(m_robotDrive,
      m_limeLight,
      m_driverController::getLeftY,
      m_driverController::getLeftX);

  private final AutoRotate m_autoRotate = new AutoRotate(m_robotDrive);
  
  // A chooser for autonomous commands
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_blinkin = new BlinkinSubsystem(0);
    // Configure the button bindings
    configureButtonBindings();
    configureAutonomousChooser();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                MathUtil.applyDeadband(m_driverController.getLeftY(), 0.1),
                MathUtil.applyDeadband(m_driverController.getLeftX(), 0.1),
                MathUtil.applyDeadband(m_driverController.getRightX(), 0.1),
                m_driverController.getPOV(),
                true),
            m_robotDrive));
    ShuffleboardTab tab = Shuffleboard.getTab("Drive System");
    tab.addBoolean("Target Mode", m_targetDrive::isScheduled);
    tab.addBoolean("Target Lock", m_limeLight::valid);
    m_limeLight.disableLED();
  }

  private void configureAutonomousChooser() {
    // Add commands to the autonomous command chooser
    m_chooser.setDefaultOption("Path Planner", getPathPlannerCommand());
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

  }

  private void configureTriggers() {
    Trigger turboTrigger = new Trigger(m_robotDrive::getTurboMode);
    turboTrigger.whenActive(() -> m_blinkin.set(BlinkinSubsystem.Color.RED.value));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    //return m_chooser.getSelected();
    return AutonomousCommandHelper.getSimplAutonomousCommand(m_robotDrive);
  }

  public Command getPathPlannerCommand() {
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    PathPlannerTrajectory examplePath = PathPlanner.loadPath("testPath2", 1, 5);
    /*
     * TrajectoryGenerator.generateTrajectory(
     * // Start at the origin facing the +X direction
     * new Pose2d(0, 0, new Rotation2d(0)),
     * // Pass through these two interior waypoints, making an 's' curve path
     * List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
     * // End 3 meters straight ahead of where we started, facing forward
     * new Pose2d(3, 0, new Rotation2d(0)),
     * config);
     */

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    PPSwerveControllerCommand swerveControllerCommand = new PPSwerveControllerCommand(examplePath,
        m_robotDrive::getPose,
        DriveConstants.kDriveKinematics,
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
   // m_robotDrive.resetOdometry(examplePath.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, -1, false));

  }


  public void periodic() {
      if (m_targetDrive.isScheduled() && !m_limeLight.valid()) {
        m_driverController.setRumble(RumbleType.kLeftRumble, 1.0);
      } else {
        m_driverController.setRumble(RumbleType.kLeftRumble, 0.0);
      }
      if (m_robotDrive.getTurboMode()) {
        m_blinkin.set(BlinkinSubsystem.Color.RED.value);
      } else {
        if (m_targetDrive.isScheduled()) {
          m_blinkin.set(BlinkinSubsystem.Color.YELLOW.value);
        } else {
          m_blinkin.set(BlinkinSubsystem.Color.BLUE.value);
        }
      }
  }
}
