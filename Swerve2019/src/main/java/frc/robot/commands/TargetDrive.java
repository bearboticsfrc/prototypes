// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.LimeLight;
import frc.robot.Constants.ModuleConstants;
import frc.robot.subsystems.DriveSubsystem;


public class TargetDrive extends CommandBase { 
  private DriveSubsystem m_driveSubsystem;
  private LimeLight m_limeLight;

  private DoubleSupplier m_xSupplier;
  private DoubleSupplier m_ySupplier;

  private final PIDController m_turnPIDController =
    new PIDController(ModuleConstants.kPTargetTurn, 0, 0);


  /** Creates a new TargetDrive. */
  public TargetDrive(DriveSubsystem driveSubsystem,
                     LimeLight limeLight,
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
    m_limeLight.setLEDs(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_limeLight.Update_Limelight_Tracking();

    double degreesToTurn = m_limeLight.m_LimelightSteerCommand;
    
    degreesToTurn = MathUtil.applyDeadband(degreesToTurn, 0.5);

    double setPoint = m_driveSubsystem.getHeading() + degreesToTurn;

    double turnOutput = m_turnPIDController.calculate(m_driveSubsystem.getHeading(), setPoint);

    SmartDashboard.putNumber("Turn Output", turnOutput);
    SmartDashboard.putNumber("degrees to turn", degreesToTurn);
    SmartDashboard.putNumber("setpoint", setPoint);

    m_driveSubsystem.drive(
      -m_ySupplier.getAsDouble(),
      -m_xSupplier.getAsDouble(),
      -turnOutput,
      -1,
      true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_limeLight.setLEDs(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
