package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveModule;


public class TestEncodersCommand extends CommandBase {

    private final SwerveModule m_frontLeft =
    new SwerveModule(
        DriveConstants.kFrontLeftDriveMotorPort,
        DriveConstants.kFrontLeftTurningMotorPort,
        DriveConstants.kFrontLeftDriveEncoderPorts,
        DriveConstants.kFrontLeftTurningEncoderPorts,
        DriveConstants.kFrontLeftDriveEncoderReversed,
        DriveConstants.kFrontLeftTurningEncoderReversed);

    public TestEncodersCommand() {
      }
    
      @Override
      public void initialize() {

      }
    
      @Override
      public void execute() {
        m_frontLeft.getState();
      }



      @Override
      public boolean isFinished() {
        return true;
      }  
}
