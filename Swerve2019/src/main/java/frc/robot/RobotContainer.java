// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.XboxController;
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
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

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

  private final LimeLight m_limeLight = new LimeLight();

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
    // Configure the button bindings
    configureButtonBindings();
    configureAutonomousChooser();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -m_driverController.getLeftY(),
                -m_driverController.getLeftX(),
                -m_driverController.getRightX(),
                m_driverController.getPOV(),
                true),
            m_robotDrive));
    ShuffleboardTab tab = Shuffleboard.getTab("Drive System");
    tab.addBoolean("Target Mode", m_targetDrive::isScheduled);
    tab.addBoolean("Target Lock", m_limeLight::hasTargetLock);
    m_limeLight.setLEDs(false);
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

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }

  public Command getPathPlannerCommand() {
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    PathPlannerTrajectory examplePath = PathPlanner.loadPath("testPath", 1, 5);
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
    m_robotDrive.resetOdometry(examplePath.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, -1, false));

  }
}
