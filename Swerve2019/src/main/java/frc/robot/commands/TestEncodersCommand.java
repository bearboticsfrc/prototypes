package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveModule;


public class TestEncodersCommand extends CommandBase {

    private final SwerveModule m_frontLeft = // front right
    new SwerveModule(
        DriveConstants.kFrontLeftDriveMotorPort,
        DriveConstants.kFrontLeftTurningMotorPort,
        DriveConstants.kFrontLeftTurningInputPort,
        DriveConstants.kFrontLeftDriveEncoderReversed,
        DriveConstants.kFrontLeftTurningEncoderReversed,
        DriveConstants.kFrontLeftZeroAngle);

    private final SwerveModule m_rearLeft = // rear left
        new SwerveModule(
            DriveConstants.kRearLeftDriveMotorPort,
            DriveConstants.kRearLeftTurningMotorPort,
            DriveConstants.kRearLeftTurningInputPort,
            DriveConstants.kRearLeftDriveEncoderReversed,
            DriveConstants.kRearLeftTurningEncoderReversed,
            DriveConstants.kRearLeftZeroAngle);
  
    private final SwerveModule m_frontRight =
        new SwerveModule(
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightTurningInputPort,
            DriveConstants.kFrontRightDriveEncoderReversed,
            DriveConstants.kFrontRightTurningEncoderReversed,
            DriveConstants.kFrontRightZeroAngle);
  
    private final SwerveModule m_rearRight =
        new SwerveModule(
            DriveConstants.kRearRightDriveMotorPort,
            DriveConstants.kRearRightTurningMotorPort,
            DriveConstants.kRearRightTurningInputPort,
            DriveConstants.kRearRightDriveEncoderReversed,
            DriveConstants.kRearRightTurningEncoderReversed,
            DriveConstants.kRearRightZeroAngle);
  
    public TestEncodersCommand(SubsystemBase subsystem) {
      addRequirements(subsystem); 
      }
    
      @Override
      public void initialize() {

      }
    
      @Override
      public void execute() {
        m_frontRight.getState();
      }


 
}
