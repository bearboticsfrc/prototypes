package frc.robot;

import java.util.List;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.PathDebugCommand;
import frc.robot.commands.PathPlannerDebugCommand;
import frc.robot.subsystems.DriveSubsystem;

public class AutonomousCommandHelper {

    public static final boolean kDebugMode = false;

    public static Command getSimplAutonomousCommand(DriveSubsystem driveSubsystem) {
        // 1. Create trajectory settings
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                        .setKinematics(DriveConstants.kDriveKinematics);
    
        // 2. Generate trajectory
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)),
                List.of(
                        new Translation2d(1, 0),
                        new Translation2d(2, 0)),
                new Pose2d(3, 0, Rotation2d.fromDegrees(0)),
                trajectoryConfig);

        if (kDebugMode) {
            double trajectoryLength = trajectory.getTotalTimeSeconds();
            System.out.println("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Trajectory total time = " + trajectoryLength);
            System.out.println(
                    "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Trajectory states size = " + trajectory.getStates().size());
            List<State> states = trajectory.getStates();

            for (State state : states) {
                System.out.println("%%%%%%%%%%%%%%%%%%%%%%%%%%% state: " + state.toString());
            }
        }
    
        // 3. Define PID controllers for tracking trajectory
        PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
        PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
        ProfiledPIDController thetaController = new ProfiledPIDController(
                AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    
        // 4. Construct command to follow trajectory
        SwerveControllerCommand swerveControllerCommand 
          = new SwerveControllerCommand(
                trajectory,
                driveSubsystem::getPose,
                DriveConstants.kDriveKinematics,
                xController,
                yController,
                thetaController,
                driveSubsystem::setModuleStates,
                driveSubsystem);

        Command autoCommand = swerveControllerCommand;
        if (kDebugMode) {
            PathDebugCommand pathDebugCommand = new PathDebugCommand(trajectory, driveSubsystem::getPose);
            ParallelCommandGroup parallelCommandGroup = new ParallelCommandGroup(swerveControllerCommand,
                                                                                 pathDebugCommand);
            autoCommand = parallelCommandGroup;
        }
        // 5. Add some init and wrap-up, and return everything
        return new SequentialCommandGroup(
                new InstantCommand(() -> driveSubsystem.resetOdometry(trajectory.getInitialPose())),
                autoCommand,
                new InstantCommand(() -> driveSubsystem.stop()));
    }
    




    public static Command getPathPlannerCommand(DriveSubsystem driveSubsystem) {
        // An example trajectory to follow. All units in meters.
        PathPlannerTrajectory pathPlannerTrajectory = PathPlanner.loadPath("simple2", .5, 1);

        if (kDebugMode) {
            double trajectoryLength = pathPlannerTrajectory.getTotalTimeSeconds();
            System.out.println("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Trajectory total time = " + trajectoryLength);
            System.out.println("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Trajectory states size = "
                    + pathPlannerTrajectory.getStates().size());
            List<State> states = pathPlannerTrajectory.getStates();

            for (State state : states) {
                PathPlannerState pState = (PathPlannerState) state;
                System.out.println("%%%%%%%%%%%%%%%%%%%%%%%%%%% holonomicRotation: " + pState.holonomicRotation
                        + " state: " + pState.toString());
            }
        }
    
        var thetaController = new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    
        PPSwerveControllerCommand swerveControllerCommand = new PPSwerveControllerCommand(pathPlannerTrajectory,
                driveSubsystem::getPose,
                DriveConstants.kDriveKinematics,
                new PIDController(AutoConstants.kPXController, 0, 0),
                new PIDController(AutoConstants.kPYController, 0, 0),
                thetaController,
                driveSubsystem::setModuleStates,
                driveSubsystem);

        Command autoCommand = swerveControllerCommand;

        if (kDebugMode) {
            PathPlannerDebugCommand pathDebugCommand = new PathPlannerDebugCommand(pathPlannerTrajectory,
                                                                                   driveSubsystem::getPose);
            ParallelCommandGroup parallelCommandGroup = new ParallelCommandGroup(swerveControllerCommand,
                                                                                 pathDebugCommand);
            autoCommand = parallelCommandGroup;
        }

        // Run path following command, then stop at the end.
        return new SequentialCommandGroup(
            new InstantCommand(() -> driveSubsystem.resetOdometry(new Pose2d(pathPlannerTrajectory.getInitialState().poseMeters.getTranslation(),
                                                                             pathPlannerTrajectory.getInitialState().holonomicRotation))),
                autoCommand,
            new InstantCommand(() -> driveSubsystem.stop()),
            new InstantCommand(() -> driveSubsystem.headingOffest(pathPlannerTrajectory.getInitialState().holonomicRotation.getDegrees())));    
      }
}
