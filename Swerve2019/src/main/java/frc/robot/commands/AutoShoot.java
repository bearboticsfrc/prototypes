// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoShoot extends CommandBase {
  private final DriveSubsystem m_driveSubsystem;
  private final LimelightSubsystem m_limeLight;
  private final ShooterSubsystem m_shooter;

  private final PIDController m_turnPIDController = new PIDController(AutoConstants.kPTargetTurn,
      AutoConstants.kITargetTurn, AutoConstants.kDTargetTurn);

  /** Creates a new TargetDrive. */
  public AutoShoot(DriveSubsystem driveSubsystem,
      LimelightSubsystem limeLight,
      ShooterSubsystem shooter) {

    m_driveSubsystem = driveSubsystem;
    m_limeLight = limeLight;
    m_shooter = shooter;

    addRequirements(m_driveSubsystem, m_limeLight, m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_limeLight.enableLED();
    m_shooter.prepare();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double degreesToTurn = m_limeLight.getX();

    degreesToTurn = (Math.abs(degreesToTurn) < 0.5) ? 0.0 : degreesToTurn;

    if (degreesToTurn == 0.0) {
      // take the shot
      m_shooter.shoot();
    } else {
      double setPoint = m_driveSubsystem.getHeading() + degreesToTurn;

      double turnOutput = m_turnPIDController.calculate(m_driveSubsystem.getHeading(), setPoint);

      m_driveSubsystem.drive(0.0, 0.0, turnOutput, true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_limeLight.disableLED();
    m_shooter.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_shooter.finishedShooting();
  }
}
