package frc.robot.commands;

import java.util.function.Supplier;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;


/**
 * Command to run in parallel with a trajectory based command.
 * Prints the current trajectory point and the odomentry point.
 */
public class PathPlannerDebugCommand extends CommandBase {

    private PathPlannerTrajectory m_trajectory;
    private Supplier<Pose2d> m_poseSupplier;
    private final Timer m_timer = new Timer();

    public PathPlannerDebugCommand(PathPlannerTrajectory trajectory, Supplier<Pose2d> poseSupplier) {
        this.m_trajectory = trajectory;
        this.m_poseSupplier = poseSupplier;
    }

    @Override
    public void initialize() {
        m_timer.reset();
        m_timer.start();
    }

    @Override
    public void execute() {
        double curTime = m_timer.get();

        var desiredState = (PathPlannerState) m_trajectory.sample(curTime);
        Pose2d pose = m_poseSupplier.get();

        System.out.println("trajectory[" + desiredState.poseMeters.getX() + "," 
                                         + desiredState.poseMeters.getY() + "," 
                                         + desiredState.poseMeters.getRotation().getDegrees() + "," 
                                         + desiredState.holonomicRotation.getDegrees() + "]"
                + " actual[" + pose.getX() + "," + pose.getY() + "," + pose.getRotation().getDegrees() + "]");
    }

    @Override
    public void end(boolean interrupted) {
        m_timer.stop();
    }

    @Override
    public boolean isFinished() {
        return m_timer.hasElapsed(m_trajectory.getTotalTimeSeconds());
    }

}
