// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;


public class TargetDrive extends CommandBase { 
  private DriveSubsystem m_driveSubsystem;
  private LimelightSubsystem m_limeLight;

  private DoubleSupplier m_xSupplier;
  private DoubleSupplier m_ySupplier;

  private final PIDController m_turnPIDController =
    new PIDController(AutoConstants.kPTargetTurn, AutoConstants.kITargetTurn, AutoConstants.kDTargetTurn);


  /** Creates a new TargetDrive. */
  public TargetDrive(DriveSubsystem driveSubsystem,
                     LimelightSubsystem limeLight,
                     DoubleSupplier ySupplier,
                     DoubleSupplier xSupplier) {

    m_driveSubsystem = driveSubsystem;
    m_limeLight = limeLight;
    m_xSupplier = xSupplier;
    m_ySupplier = ySupplier;

    addRequirements(m_driveSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_limeLight.enableLED();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //m_limeLight.Update_Limelight_Tracking();

    double degreesToTurn = m_limeLight.getX();
    
    degreesToTurn = (Math.abs(degreesToTurn) < 0.5) ? 0.0 : degreesToTurn; 

    double setPoint = m_driveSubsystem.getHeading() + degreesToTurn;

    double turnOutput = m_turnPIDController.calculate(m_driveSubsystem.getHeading(), setPoint);

    SmartDashboard.putNumber("Turn Output", turnOutput);
    SmartDashboard.putNumber("degrees to turn", degreesToTurn);
    SmartDashboard.putNumber("setpoint", setPoint);

    m_driveSubsystem.drive(
      MathUtil.applyDeadband(m_ySupplier.getAsDouble(), 0.1),
      MathUtil.applyDeadband(m_xSupplier.getAsDouble(), 0.1),
      turnOutput,
      -1,
      true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_limeLight.disableLED();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
