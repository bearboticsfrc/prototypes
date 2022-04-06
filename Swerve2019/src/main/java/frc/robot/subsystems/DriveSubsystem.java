// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU.PigeonState;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableEntry;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.PivotPoint;

public class DriveSubsystem extends MeasuredSubsystem {
  // Robot swerve modules
  private final SwerveModule m_frontLeft;
  private final SwerveModule m_backLeft;
  private final SwerveModule m_frontRight;
  private final SwerveModule m_backRight;

  // The gyro sensor
  private final WPI_PigeonIMU m_gyro = new WPI_PigeonIMU(10);
  private double m_gyroOffetDegrees = 0.0;

  private double kMaxSpeed = 3; // 3 meters per second
  private double m_maxSpeed = kMaxSpeed;
  private boolean m_turboMode = false;
  private boolean m_fieldRelativeMode = true;

  private PivotPoint m_pivotPoint = PivotPoint.CENTER;

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, m_gyro.getRotation2d());

  ShuffleboardTab tab = Shuffleboard.getTab("Drive System");
  ShuffleboardTab swerveModulesTab = Shuffleboard.getTab("Swerve Modules");

  NetworkTableEntry maxSpeedEntry = tab.add("Drive Speed", kMaxSpeed)
      .withWidget(BuiltInWidgets.kNumberSlider)
      .withSize(2, 1)
      .withProperties(Map.of("min", 0, "max", kMaxSpeed))
      .getEntry();

  private final SlewRateLimiter xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
  private final SlewRateLimiter yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
  private final SlewRateLimiter turningLimiter = new SlewRateLimiter(
      DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);

  private final BlinkinSubsystem m_blinkin;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem(BlinkinSubsystem blinkin) {
    System.out.println("^^^^^^^^^^^^^^^^^^^^^ Gyro is " + (this.isGyroReady()? "Ready":"Not Ready"));
    
    m_blinkin = blinkin;
    if ( this.isGyroReady() ) {
      m_blinkin.set(BlinkinSubsystem.Color.GREEN);
    } else {
      m_blinkin.set(BlinkinSubsystem.Color.ORANGE);
    }
    m_frontLeft = new SwerveModule(
        DriveConstants.kFrontLeftDriveMotorPort,
        DriveConstants.kFrontLeftTurningMotorPort,
        DriveConstants.kFrontLeftTurningInputPort,
        DriveConstants.kFrontLeftDriveEncoderReversed,
        DriveConstants.kFrontLeftTurningEncoderReversed,
        DriveConstants.kFrontLeftZeroAngle,
        swerveModulesTab.getLayout("Front Left", BuiltInLayouts.kList)
            .withSize(2, 4)
            .withPosition(0, 0),
        "FL");

    m_backLeft = new SwerveModule(
        DriveConstants.kBackLeftDriveMotorPort,
        DriveConstants.kBackLeftTurningMotorPort,
        DriveConstants.kBackLeftTurningInputPort,
        DriveConstants.kBackLeftDriveEncoderReversed,
        DriveConstants.kBackLeftTurningEncoderReversed,
        DriveConstants.kBackLeftZeroAngle,
        swerveModulesTab.getLayout("Back Left", BuiltInLayouts.kList)
            .withSize(2, 4)
            .withPosition(2, 0),
        "BL");

    m_frontRight = new SwerveModule(
        DriveConstants.kFrontRightDriveMotorPort,
        DriveConstants.kFrontRightTurningMotorPort,
        DriveConstants.kFrontRightTurningInputPort,
        DriveConstants.kFrontRightDriveEncoderReversed,
        DriveConstants.kFrontRightTurningEncoderReversed,
        DriveConstants.kFrontRightZeroAngle,
        swerveModulesTab.getLayout("Front Right", BuiltInLayouts.kList)
            .withSize(2, 4)
            .withPosition(4, 0),
        "FR");

    m_backRight = new SwerveModule(
        DriveConstants.kBackRightDriveMotorPort,
        DriveConstants.kBackRightTurningMotorPort,
        DriveConstants.kBackRightTurningInputPort,
        DriveConstants.kBackRightDriveEncoderReversed,
        DriveConstants.kBackRightTurningEncoderReversed,
        DriveConstants.kBackRightZeroAngle,
        swerveModulesTab.getLayout("Back Right", BuiltInLayouts.kList)
            .withSize(2, 4)
            .withPosition(6, 0),
        "BR");

    zeroHeading();
    //tab.addNumber("Gyro Angle", this::getHeading).withWidget(BuiltInWidgets.kGyro);
    tab.addNumber("Pose X", () -> m_odometry.getPoseMeters().getX());
    tab.addNumber("Pose Y", () -> m_odometry.getPoseMeters().getY());
    tab.addNumber("Pose rot", () -> m_odometry.getPoseMeters().getRotation().getDegrees());
  }

  public boolean isGyroReady() {
    return m_gyro.getState() == PigeonState.Ready;
  }
  
  public void setTurboMode(boolean mode) {
    if (mode) {
      m_turboMode = true;
      m_maxSpeed = Math.min(m_maxSpeed*2.0, kMaxSpeed);
      m_blinkin.set(BlinkinSubsystem.Color.RED);
    } else {
      m_turboMode = false;
      m_maxSpeed /= 2;
      m_blinkin.set(m_blinkin.getPreviousColor());
    }
    maxSpeedEntry.setDouble(m_maxSpeed);
  }

  public boolean getTurboMode() {
    return m_turboMode;
  }

  public void setFieldRelative(boolean mode) {
    m_fieldRelativeMode = mode;
    if (!mode) {
      m_blinkin.set(BlinkinSubsystem.Color.HOT_PINK);
    } else {
      m_blinkin.set(m_blinkin.getPreviousColor());
    }
  }

  public boolean getFieldRelative() {
    return m_fieldRelativeMode;
  }


  @Override
  public void monitored_periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        m_gyro.getRotation2d().plus(Rotation2d.fromDegrees(m_gyroOffetDegrees)),
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_backLeft.getState(),
        m_backRight.getState());
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
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose, Rotation2d angle) {
    m_odometry.resetPosition(pose, angle);
  }

  /**
   * Drive robot using Joystick inputs, default to CENTER pivot, and sets
   * field relative to the current fieldRelativeMode setting.
   * 
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   */
  public void drive(double xSpeed, double ySpeed, double rot) {
    this.drive(xSpeed, ySpeed, rot, PivotPoint.CENTER, m_fieldRelativeMode);
  }

  /**
   * Drive robot using Joystick inputs, sets field relative to the current fieldRelativeMode setting.
   * 
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   */
  public void drive(double xSpeed, double ySpeed, double rot, PivotPoint pivotPoint) {
    this.drive(xSpeed, ySpeed, rot, pivotPoint, m_fieldRelativeMode);
  }

  /**
   * Drive robot using Joystick inputs, default to CENTER pivot.
   * 
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative )  {
    this.drive(xSpeed, ySpeed, rot, PivotPoint.CENTER, fieldRelative);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param PivotPoint    Where to pivot for turning.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rot, PivotPoint pivotPoint, boolean fieldRelative) {
    xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kMaxSpeedMetersPerSecond;
    ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kMaxSpeedMetersPerSecond;
    rot = turningLimiter.calculate(rot) * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond
        * DriveConstants.kMaxSpeedMetersPerSecond;
    setPivotPoint(pivotPoint);
    // ShuffleboardTab tab = Shuffleboard.getTab("Drive System");
    // tab.add("controller x", xSpeed);
    // tab.add("controller y", ySpeed);
    // tab.add("controller rot", rot);
    // SmartDashboard.putNumber("rot", rot);

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d().plus(Rotation2d.fromDegrees(m_gyroOffetDegrees)))
            : new ChassisSpeeds(xSpeed, ySpeed, rot),
        m_pivotPoint.get());
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

  public void stop() {
    m_frontLeft.stop();
    m_frontRight.stop();
    m_backLeft.stop();
    m_backRight.stop();
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
    m_gyroOffetDegrees = 0.0;
    m_gyro.reset();
  }

 /** Zeroes the heading of the robot. */
 public void headingOffest(double offsetDegrees) {
   System.out.println("!!!!!!!!!!! Adding " + offsetDegrees);
   m_gyroOffetDegrees = offsetDegrees;
 // m_gyro.addFusedHeading(offsetDegrees);
}

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    double temp = m_gyro.getRotation2d().getDegrees();
    temp -= Math.floor(temp / 360.0) * 360.0;
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
    m_frontLeft.outputToSmartDashboard();
    m_frontRight.outputToSmartDashboard();
    m_backLeft.outputToSmartDashboard();
    m_backRight.outputToSmartDashboard();
  }

  public void setPivotPoint(PivotPoint pivotPoint) {
    m_pivotPoint = pivotPoint;
  }
}
