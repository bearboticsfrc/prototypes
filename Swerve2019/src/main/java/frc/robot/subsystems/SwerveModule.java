// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.AnalogInput;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {
  private final CANSparkMax m_driveMotor;
  private final WPI_VictorSPX m_turningMotor;

  private final RelativeEncoder m_driveEncoder;

  private final AnalogInput m_turningEncoder;
  private double m_zeroAngle;

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

  /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorChannel ID for the drive motor.
   * @param turningMotorChannel ID for the turning motor.
   */
  public SwerveModule(
      int driveMotorChannel,
      int turningMotorChannel,
      int turningAnalogPort,
      boolean driveEncoderReversed,
      boolean turningEncoderReversed,
      double zeroAngle) {
    m_driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
    m_turningMotor = new WPI_VictorSPX(turningMotorChannel);
    m_turningMotor.setNeutralMode(NeutralMode.Coast);
    //this.m_driveEncoder = new Encoder(driveEncoderPorts[0], driveEncoderPorts[1]);
    this.m_driveEncoder = m_driveMotor.getEncoder();

    //this.m_turningInput = new AnalogInput(turningEncoderPorts[0]);
    this.m_turningEncoder = new AnalogInput(turningAnalogPort);
    this.m_zeroAngle = zeroAngle;

   // this.m_turningEncoder = new Encoder(turningEncoderPorts[0], turningEncoderPorts[1]);

    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    //m_driveEncoder.setDistancePerPulse(ModuleConstants.kDriveEncoderDistancePerPulse);

    // Set whether drive encoder should be reversed or not
    //m_driveEncoder.setReverseDirection(driveEncoderReversed);
    //m_driveEncoder.setInverted(driveEncoderReversed);


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
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
   // System.out.println("getState(): driveEncoderVelocity: " + m_driveEncoder.getVelocity() + ", turningEncoderGet: " + getAngle()  + " raw: " + getRawAngle());
    return new SwerveModuleState(m_driveEncoder.getVelocity(), new Rotation2d(Math.toRadians(getAngle())));
  }

  public double getRawAngle() {
    return m_turningEncoder.getVoltage() * 360.0 / 5.0;
  }

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

    System.out.println("Desired angle: " + desiredState.angle.getDegrees() + " Current angle: " + getAngle());
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state =
        SwerveModuleState.optimize(desiredState, new Rotation2d(Math.toRadians(getAngle())));

    // Calculate the drive output from the drive PID controller.
    final double driveOutput =
        m_drivePIDController.calculate(m_driveEncoder.getVelocity(), state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final var turnOutput =
        m_turningPIDController.calculate(Math.toRadians(getAngle()), state.angle.getRadians());

    System.out.println("setDesiredState: driveOutput: " + driveOutput + ", turnOutput: " + turnOutput );

    // Calculate the turning motor output from the turning PID controller.
   // m_driveMotor.set(driveOutput);
    m_turningMotor.set(turnOutput);
  }

  /** Zeros all the SwerveModule encoders. */
  public void resetEncoders() {
    //m_driveEncoder.reset();
    m_driveEncoder.setPosition(0);
    //m_turningEncoder.reset();
  }
}
