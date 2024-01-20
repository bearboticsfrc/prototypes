/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.GenericEntry;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import static frc.robot.util.RevUtil.checkRevError;

/**
 * This is a demo program showing the use of the RobotDrive class, specifically
 * it contains the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {
  private final ShuffleboardTab DRIVE_SHUFFLEBOARD_TAB = Shuffleboard.getTab("Drive");

  private final CANSparkFlex motor = new CANSparkFlex(18, MotorType.kBrushless);
  private final CANSparkFlex motorFollower = new CANSparkFlex(19, MotorType.kBrushless);
  private RelativeEncoder motorEncoder;
  private SparkPIDController motorPidController;

  private double targetRpm = 0;
  private double p = 0.00028; 
  private double i = 0.000001;
  private double d = 0;
  private double iz = 0;
  private double ff = 0;
  private double maxOutput = 1;
  private double minOutput = -1;

  private GenericEntry targetRpmEntry = DRIVE_SHUFFLEBOARD_TAB.add("Target Velocity (RPM)", targetRpm).getEntry();
  private GenericEntry pEntry = DRIVE_SHUFFLEBOARD_TAB.add("PID - P", p).getEntry();
  private GenericEntry iEntry = DRIVE_SHUFFLEBOARD_TAB.add("PID - I", i).getEntry();
  private GenericEntry dEntry = DRIVE_SHUFFLEBOARD_TAB.add("PID - D", d).getEntry();
  private GenericEntry izEntry = DRIVE_SHUFFLEBOARD_TAB.add("PID - Iz", iz).getEntry();
  private GenericEntry ffEntry = DRIVE_SHUFFLEBOARD_TAB.add("PID FF", ff).getEntry();

  @Override
  public void robotInit() {
    setupShooterMotors();
    setupPidController();
    setupShuffleboardTab();
  }

  private void setupShooterMotors() {
    checkRevError(motor.restoreFactoryDefaults());
    checkRevError(motorFollower.restoreFactoryDefaults());
    checkRevError(motor.enableVoltageCompensation(12.0));
    motor.setInverted(false);
    checkRevError(motorFollower.enableVoltageCompensation(12.0));
    checkRevError(motor.setSmartCurrentLimit(20));
    checkRevError(motorFollower.setSmartCurrentLimit(20));
    checkRevError(motor.setIdleMode(IdleMode.kCoast));
    checkRevError(motorFollower.setIdleMode(IdleMode.kCoast));
    checkRevError(motorFollower.follow(motor, false));
    //checkRevError(motor.setClosedLoopRampRate(0.005));

    motorEncoder = motor.getEncoder();
  }

  private void setupPidController() {
    motorPidController = motor.getPIDController();

    motorPidController.setP(p);
    motorPidController.setI(i);
    motorPidController.setD(d);
    motorPidController.setIZone(iz);
    motorPidController.setFF(ff);
    motorPidController.setOutputRange(minOutput, maxOutput);
  }

  private void setupShuffleboardTab() {
    DRIVE_SHUFFLEBOARD_TAB.addNumber("Velocity (RPM)", motorEncoder::getVelocity);
    DRIVE_SHUFFLEBOARD_TAB.addNumber("Applied Output (A)", motor::getAppliedOutput);
    DRIVE_SHUFFLEBOARD_TAB.addNumber("Applied Current (A)", motor::getOutputCurrent);
    DRIVE_SHUFFLEBOARD_TAB.addNumber("Temperature (C)", motor::getMotorTemperature);
  }

   private void updateValues() {
    p = pEntry.get().getDouble();
    d = dEntry.get().getDouble();
    i = iEntry.get().getDouble();
    iz = izEntry.get().getDouble();
    ff = ffEntry.get().getDouble();
  }

  @Override
  public void teleopInit() {
    updateValues();
    setupPidController();
  }


  @Override
  public void teleopPeriodic() {
    targetRpm = targetRpmEntry.getDouble(targetRpm);

    if (targetRpm == 0) {
      motor.set(0);
      return;
    } 

    motorPidController.setReference(targetRpm, CANSparkBase.ControlType.kVelocity);
  }
}