// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableEntry;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.PivotPoint;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends MeasuredSubsystem { // implements Loggable{
  // Robot swerve modules
  private final SwerveModule m_frontLeft =
      new SwerveModule(
          DriveConstants.kFrontLeftDriveMotorPort,
          DriveConstants.kFrontLeftTurningMotorPort,
          DriveConstants.kFrontLeftTurningInputPort,
          DriveConstants.kFrontLeftDriveEncoderReversed,
          DriveConstants.kFrontLeftTurningEncoderReversed,
          DriveConstants.kFrontLeftZeroAngle,
          "FL");

  private final SwerveModule m_backLeft =
      new SwerveModule(
          DriveConstants.kBackLeftDriveMotorPort,
          DriveConstants.kBackLeftTurningMotorPort,
          DriveConstants.kBackLeftTurningInputPort,
          DriveConstants.kBackLeftDriveEncoderReversed,
          DriveConstants.kBackLeftTurningEncoderReversed,
          DriveConstants.kBackLeftZeroAngle,
          "BL");

  private final SwerveModule m_frontRight =
      new SwerveModule(
          DriveConstants.kFrontRightDriveMotorPort,
          DriveConstants.kFrontRightTurningMotorPort,
          DriveConstants.kFrontRightTurningInputPort,
          DriveConstants.kFrontRightDriveEncoderReversed,
          DriveConstants.kFrontRightTurningEncoderReversed,
          DriveConstants.kFrontRightZeroAngle,
          "FR");

  private final SwerveModule m_backRight =
      new SwerveModule(
          DriveConstants.kBackRightDriveMotorPort,
          DriveConstants.kBackRightTurningMotorPort,
          DriveConstants.kBackRightTurningInputPort,
          DriveConstants.kBackRightDriveEncoderReversed,
          DriveConstants.kBackRightTurningEncoderReversed,
          DriveConstants.kBackRightZeroAngle,
          "BR");

  // The gyro sensor
  private final WPI_PigeonIMU m_gyro = new WPI_PigeonIMU(10);

  private double kMaxSpeed = 5; // 3 meters per second
  private double m_maxSpeed = kMaxSpeed;

  public void setTurboMode(boolean mode) {
    if(mode) {
      m_maxSpeed *= 2;
    } else {
      m_maxSpeed /= 2;
    }
    maxSpeedEntry.setDouble(m_maxSpeed);
  }

  private PivotPoint m_pivotPoint = PivotPoint.CENTER;

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(DriveConstants.kDriveKinematics, m_gyro.getRotation2d());

  ShuffleboardTab tab = Shuffleboard.getTab("Drive System");

  NetworkTableEntry maxSpeedEntry =  
        tab.add("Drive Speed", kMaxSpeed)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withSize(2,1)
        .withProperties(Map.of("min", 0, "max", kMaxSpeed))
        .getEntry();
        
  private final SlewRateLimiter xLimiter =  new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
  private final SlewRateLimiter yLimiter =  new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
  private final SlewRateLimiter turningLimiter =  new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
       
  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    zeroHeading();
   // tab.add("Gyro", m_gyro).withWidget(BuiltInWidgets.kGyro);
  }

  @Override
  public void monitored_periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        m_gyro.getRotation2d(),
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_backRight.getState(),
        m_backLeft.getState());
    outputToSmartDashboard();    
    m_maxSpeed = maxSpeedEntry.getDouble(kMaxSpeed);
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public Vector2d getVelocity() {
    return new Vector2d(0.0, 0.0);
  }
  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(pose, m_gyro.getRotation2d());
  }

  

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rot, int pov, boolean fieldRelative) {
    xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kMaxSpeedMetersPerSecond;
    ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kMaxSpeedMetersPerSecond;
    rot = turningLimiter.calculate(rot) * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond * DriveConstants.kMaxSpeedMetersPerSecond;  
    //rot = turningLimiter.calculate(rot) * DriveConstants.kMaxSpeedMetersPerSecond;  
    setPivotPoint(getPivotPointByPOV(pov));
    //ShuffleboardTab tab = Shuffleboard.getTab("Drive System");
    //tab.add("controller x", xSpeed);
    //tab.add("controller y", ySpeed);
    //tab.add("controller rot", rot);
    SmartDashboard.putNumber("rot", rot);

    var swerveModuleStates =
        DriveConstants.kDriveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot), m_pivotPoint.get());
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, m_maxSpeed);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, m_maxSpeed);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_backLeft.setDesiredState(desiredStates[2]);
    m_backRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_backLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_backRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    double temp =  m_gyro.getRotation2d().getDegrees();
    temp -= Math.floor(temp/360.0) * 360.0;
    return temp;
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public void outputToSmartDashboard() {
    SmartDashboard.putNumber("Gyro", getHeading());
    m_frontLeft.outputToSmartDashboard();
    m_frontRight.outputToSmartDashboard();
    m_backLeft.outputToSmartDashboard();
    m_backRight.outputToSmartDashboard();
  }

  public PivotPoint getPivotPointByPOV( int pov ) {
    switch (pov) {
      case 0:
        return PivotPoint.FRONT_LEFT;
      case 90:
        return PivotPoint.FRONT_RIGHT;
      case 180:
        return PivotPoint.BACK_RIGHT;
      case 270:
        return PivotPoint.BACK_LEFT;
      default:
        return PivotPoint.CENTER;
    }
  }
  
  public void setPivotPoint(PivotPoint pivotPoint) {
    m_pivotPoint = pivotPoint;
  }
}
