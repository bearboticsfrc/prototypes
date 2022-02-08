// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.LayoutType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.Constants.DriveConstants;

public class SwerveModule { 
  private final CANSparkMax m_driveMotor;
  private final WPI_VictorSPX m_turningMotor;

  private final RelativeEncoder m_driveEncoder;

  private final AnalogInput m_turningEncoder;
  
  private double m_zeroAngle;
  private String m_moduleName;

  private final PIDController m_drivePIDController =
      new PIDController(SwerveModuleConstants.kPModuleDriveController, 0, 0);

  // Using a TrapezoidProfile PIDController to allow for smooth turning
  private final ProfiledPIDController m_turningPIDController =
      new ProfiledPIDController(
          SwerveModuleConstants.kPModuleTurningController,
          0,
          0,
          new TrapezoidProfile.Constraints(
              SwerveModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond,
              SwerveModuleConstants.kMaxModuleAngularAccelerationRadiansPerSecondSquared));

  private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(1,.5);
  /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorChannel ID for the drive motor.
   * @param turningMotorChannel ID for the turning motor.
   * @param turningAnalogPort serial port for the analog input
   * @param driveEncoderReversed if the drive encoder should be reversed.
   * @param zeroAngle default offset of the encoder to make the wheel at the correct starting position.
   * @param moduleName Name (FL, FR, BL, BR)
   */
  public SwerveModule(
      int driveMotorChannel,
      int turningMotorChannel,
      int turningAnalogPort,
      boolean driveEncoderReversed,
      boolean turningEncoderReversed,
      double zeroAngle,
      ShuffleboardLayout container,
      String moduleName) {
    m_driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
    m_driveMotor.restoreFactoryDefaults();
    if (m_driveMotor.setIdleMode(IdleMode.kBrake) != REVLibError.kOk){
      SmartDashboard.putString("Idle Mode", "Error");
    }

    m_turningMotor = new WPI_VictorSPX(turningMotorChannel);
    m_turningMotor.setNeutralMode(NeutralMode.Coast);

    this.m_driveEncoder = m_driveMotor.getEncoder();

    this.m_turningEncoder = new AnalogInput(turningAnalogPort);
    
    this.m_zeroAngle = zeroAngle;
    this.m_moduleName = moduleName;

    m_driveEncoder.setVelocityConversionFactor(SwerveModuleConstants.kDriveEncoderRPM2MeterPerSec); // RPM to units per second
    m_driveEncoder.setPositionConversionFactor(SwerveModuleConstants.kDriveEncoderRotationsPerMeter);

    m_driveMotor.setInverted(driveEncoderReversed);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

    if (SwerveModuleConstants.kSwerveModuleDebugMode) {
      //ShuffleboardTab tab = Shuffleboard.getTab("Drive System");
      container.addNumber(String.format("%s angle", m_moduleName), this::getAngle);
      container.addNumber(String.format("%s velocity", m_moduleName), m_driveEncoder::getVelocity);
      container.addNumber(String.format("%s drive current", m_moduleName), m_driveMotor::getOutputCurrent);
      container.addNumber(String.format("%s desired angle", m_moduleName), this::getDesiredAngleDegrees);
      container.addNumber(String.format("%s desired speed", m_moduleName), this::getDesiredSpeedMetersPerSecond);
      container.addNumber(String.format("%s drive value", m_moduleName), this::getCalculatedDriveValue);
    }
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(m_driveEncoder.getVelocity(), Rotation2d.fromDegrees(getAngle()));
  }

  public double getRawAngle() {
    return m_turningEncoder.getVoltage() * 360.0 / 5.0;
  }

  public double getAngle() {
    double temp =  getRawAngle() - m_zeroAngle;

    temp -= Math.floor(temp/360.0) * 360.0;

    return temp;
  }

  private SwerveModuleState m_desiredState = new SwerveModuleState();

  public double getDesiredSpeedMetersPerSecond() {
    return m_desiredState.speedMetersPerSecond;
  }

  public double getDesiredAngleDegrees() {
    return m_desiredState.angle.getDegrees();
  }

  public SwerveModuleState getOptimizedState() {
    return SwerveModuleState.optimize(m_desiredState, Rotation2d.fromDegrees(getAngle()));
  }

  public double getCalculatedDriveValue() {
    return getOptimizedState().speedMetersPerSecond / DriveConstants.kMaxSpeedMetersPerSecond;
  }
  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    m_desiredState = desiredState;
    //if (Math.abs(desiredState.speedMetersPerSecond) < 0.001) {
    //  stop();
    //  return;
    //}

    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state = getOptimizedState();

    // Calculate the drive output from the drive PID controller.
    final double driveOutput =
        m_drivePIDController.calculate(m_driveEncoder.getVelocity(), state.speedMetersPerSecond);

    final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);

    //double driveValue = MathUtil.clamp(driveOutput + state.speedMetersPerSecond, -0.2, 0.2);    

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput =
        m_turningPIDController.calculate(Math.toRadians(getAngle()), state.angle.getRadians());
        
    m_driveMotor.set(getCalculatedDriveValue());
    m_turningMotor.set(turnOutput);
  }

  public void stop() {
    m_driveMotor.set(0);
    m_turningMotor.set(0);
}

  /** Zeros all the SwerveModule encoders. */
  public void resetEncoders() {
    //m_driveEncoder.reset();
    //m_turningEncoder.reset();
    m_driveEncoder.setPosition(0);
  }

  public void outputToSmartDashboard() {
  }

}
