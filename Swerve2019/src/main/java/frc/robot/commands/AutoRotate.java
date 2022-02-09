// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants.PivotPoint;
import frc.robot.subsystems.DriveSubsystem;

public class AutoRotate extends CommandBase {
  private DriveSubsystem m_driveSubsystem;

  private final PIDController m_turnPIDController = new PIDController(AutoConstants.kPAutoTurn, 0, 0);

  private List<Double> m_degreeSetPoints = Arrays.asList(90.0, 180.0, 270.0, 360.0);
  private int m_step = 0;
  private double m_offset = 0.0;

  /** Creates a new TargetDrive. */
  public AutoRotate(DriveSubsystem driveSubsystem) {
    m_driveSubsystem = driveSubsystem;
    addRequirements(m_driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_offset = m_driveSubsystem.getHeading();
    m_step = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double setPoint = m_degreeSetPoints.get(m_step) + m_offset;

    double turnOutput = m_turnPIDController.calculate(m_driveSubsystem.getHeading(), setPoint);

    //SmartDashboard.putNumber("Turn Output", turnOutput);
    //SmartDashboard.putNumber("setpoint", setPoint);

    m_driveSubsystem.drive(
        0.0,
        0.0,
        -turnOutput,
        PivotPoint.getByPOV(m_degreeSetPoints.get(m_step).intValue()),
        true);

    if (Math.abs(m_driveSubsystem.getHeading() - setPoint) < 0.5) {
      m_step++;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_step >= m_degreeSetPoints.size();
  }
}
