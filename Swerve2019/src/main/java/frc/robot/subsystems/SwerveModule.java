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
import edu.wpi.first.math.MathUtil;
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
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.DriveConstants;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class SwerveModule { // implements Loggable {
  private final CANSparkMax m_driveMotor;
  private final WPI_VictorSPX m_turningMotor;

  private final RelativeEncoder m_driveEncoder;

  private final AnalogInput m_turningEncoder;
  
  @Log
  private double m_zeroAngle;
  private String m_moduleName;

  private final PIDController m_drivePIDController =
      new PIDController(ModuleConstants.kPModuleDriveController, 0, 0);

  // Using a TrapezoidProfile PIDController to allow for smooth turning
  private final ProfiledPIDController m_turningPIDController =
      new ProfiledPIDController(
          ModuleConstants.kPModuleTurningController,
          0,
          0,
          new TrapezoidProfile.Constraints(
              ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond,
              ModuleConstants.kMaxModuleAngularAccelerationRadiansPerSecondSquared));

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
      String moduleName) {
    m_driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
    m_driveMotor.restoreFactoryDefaults();
    if (m_driveMotor.setIdleMode(IdleMode.kCoast) != REVLibError.kOk){
      SmartDashboard.putString("Idle Mode", "Error");
    }

    m_turningMotor = new WPI_VictorSPX(turningMotorChannel);
    m_turningMotor.setNeutralMode(NeutralMode.Coast);

    this.m_driveEncoder = m_driveMotor.getEncoder();

    this.m_turningEncoder = new AnalogInput(turningAnalogPort);
    
    this.m_zeroAngle = zeroAngle;
    this.m_moduleName = moduleName;

    m_driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec); // RPM to units per second
    m_driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRotationsPerMeter);

    // Set whether drive encoder should be reversed or not
    m_driveEncoder.setInverted(driveEncoderReversed);

    // Set the distance (in this case, angle) per pulse for the turning encoder.
    // This is the the angle through an entire rotation (2 * pi) divided by the
    // encoder resolution.
    // m_turningEncoder.setDistancePerPulse(ModuleConstants.kTurningEncoderDistancePerPulse);
    // m_turningEncoder.setDistancePerRotation(ModuleConstants.kTurningEncoderDistancePerPulse);

    // Set whether turning encoder should be reversed or not
    //m_turningEncoder.setReverseDirection(turningEncoderReversed);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

    //ShuffleboardTab tab = Shuffleboard.getTab("Drive System");
    //tab.addNumber(String.format("%s angle", m_moduleName), this::getAngle);
    //SmartDashboard.putNumber(String.format("%s module drive distance", m_moduleName), module.getCurrentDistance());
    //SmartDashboard.putString(String.format("%s module position", m_moduleName), module.getCurrentPosition().toString());
    //tab.addNumber(String.format("%s velocity", m_moduleName), m_driveEncoder::getVelocity);
    //tab.addNumber(String.format("%s drive current", m_moduleName), m_driveMotor::getOutputCurrent);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(m_driveEncoder.getVelocity(), new Rotation2d(Math.toRadians(getAngle())));
  }

  public double getRawAngle() {
    return m_turningEncoder.getVoltage() * 360.0 / 5.0;
  }

  @Log(name = "Angle")
  public double getAngle() {
    double temp =  getRawAngle() - m_zeroAngle;

    temp -= Math.floor(temp/360.0) * 360.0;

    return temp;
  }
  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    //if (Math.abs(desiredState.speedMetersPerSecond) < 0.001) {
    //  stop();
    //  return;
    //}


    //desiredState.angle = Rotation2d.fromDegrees(0);
    //desiredState.speedMetersPerSecond = 0.0;
    //ShuffleboardTab tab = Shuffleboard.getTab("Drive System");

    //SmartDashboard.putNumber(String.format("%s desired angle", m_moduleName), desiredState.angle.getDegrees());
    //SmartDashboard.putNumber(String.format("%s angle", m_moduleName), getAngle());
    //SmartDashboard.putNumber(String.format("%s raw angle", m_moduleName), getRawAngle());
    //tab.addNumber(String.format("%s desired speed", m_moduleName), desiredState.speedMetersPerSecond);
    //SmartDashboard.putNumber(String.format("%s desired speed", m_moduleName), desiredState.speedMetersPerSecond);

    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state =
        SwerveModuleState.optimize(desiredState, new Rotation2d(Math.toRadians(getAngle())));

    // Calculate the drive output from the drive PID controller.
    final double driveOutput =
        m_drivePIDController.calculate(m_driveEncoder.getVelocity(), state.speedMetersPerSecond);

    final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);
    // Calculate the turning motor output from the turning PID controller.
    final var turnOutput =
        m_turningPIDController.calculate(Math.toRadians(getAngle()), state.angle.getRadians());
    
    double driveValue = MathUtil.clamp(driveOutput + state.speedMetersPerSecond, -0.2, 0.2);    
    
    driveValue = state.speedMetersPerSecond / DriveConstants.kMaxSpeedMetersPerSecond;
   // SmartDashboard.putNumber(String.format("%s drive value", m_moduleName), driveValue);
    //SmartDashboard.putNumber(String.format("%s drive feed forward", m_moduleName), driveFeedforward);
   // SmartDashboard.putNumber(String.format("%s turn output", m_moduleName), turnOutput);
    //SmartDashboard.putNumber(String.format("%s drive value", m_moduleName), driveValue);
    // Calculate the turning motor output from the turning PID controller.
    
    m_driveMotor.set(driveValue);
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

  //@Override
  //public String configureLogName() {
  //    return m_moduleName;
  //}

  //@Override
  //public LayoutType configureLayoutType() {
  //    return BuiltInLayouts.kGrid;
  //}

  //@Override
  //public int[] configureLayoutSize() {
  //    int[] size = {3,4};
  //    return size;
  //  }
}
