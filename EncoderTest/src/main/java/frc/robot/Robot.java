// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}


  
  public void addEncodersTest() {
    SRXMagEncoder encoder1 = new SRXMagEncoder(DriveConstants.kFrontLeftTurningInputPort, DriveConstants.kFrontLeftEncoderAngle);
    SRXMagEncoder encoder2 = new SRXMagEncoder(DriveConstants.kBackLeftTurningInputPort, DriveConstants.kBackLeftEncoderAngle);
    SRXMagEncoder encoder3 = new SRXMagEncoder(DriveConstants.kFrontRightTurningInputPort, DriveConstants.kFrontRightEncoderAngle);
    SRXMagEncoder encoder4 = new SRXMagEncoder(DriveConstants.kBackRightTurningInputPort, DriveConstants.kBackRightEncoderAngle);

    ShuffleboardTab tab = Shuffleboard.getTab("Encoders");

    tab.addNumber("FL 1", () -> encoder1.getAbsolutePosition());
    tab.addNumber("FL 2", () -> encoder1.getAbsolutePosition2());

    tab.addNumber("BL 1", () -> encoder2.getAbsolutePosition());
    tab.addNumber("BL 2", () -> encoder2.getAbsolutePosition2());

    tab.addNumber("FR 1", () -> encoder3.getAbsolutePosition());
    tab.addNumber("FR 2", () -> encoder3.getAbsolutePosition2());

    tab.addNumber("BR 1", () -> encoder4.getAbsolutePosition());
    tab.addNumber("BR 2", () -> encoder4.getAbsolutePosition2());
  }
  
  public static final class DriveConstants {
   public static final int kFrontLeftTurningInputPort = 3;
   public static final int kBackLeftTurningInputPort = 1;
   public static final int kFrontRightTurningInputPort = 2;
   public static final int kBackRightTurningInputPort = 0;

   public static final double kFrontLeftEncoderAngle = 94.6; 
   public static final double kBackLeftEncoderAngle =  32.42;  
   public static final double kFrontRightEncoderAngle = 114.73; 
   public static final double kBackRightEncoderAngle = 292.65; 
  }

 }
