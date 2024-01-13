/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.NetworkTableEntry;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import static frc.robot.util.RevUtil.checkRevError;


import edu.wpi.first.math.filter.LinearFilter;

/**
 * This is a demo program showing the use of the RobotDrive class, specifically
 * it contains the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {
  private final ShuffleboardTab DRIVE_SHUFFLEBOARD_TAB = Shuffleboard.getTab("Drive");
  private final XboxController driverController = new XboxController(0);

  private final CANSparkFlex shooterMotor = new CANSparkFlex(16, MotorType.kBrushless);
  private final CANSparkFlex shooterMotorFollower = new CANSparkFlex(17, MotorType.kBrushless);
  private RelativeEncoder motorEncoder;
  private SparkPIDController motorPidController;

  private double p = 0.001;
  private double i = 0.00000007; 
  private double d = 0.000000008;
  private double iz = 400;
  private double ff = 0.000168;
  private double maxOutput = 1;
  private double minOutput = -1;
  private int targetRpm = 0;

  private NetworkTableEntry targetRpmEntry = DRIVE_SHUFFLEBOARD_TAB.add("targetRPM", targetRpm).getEntry();

  private Timer timer = new Timer();
  private LinearFilter filter = LinearFilter.movingAverage(20);
  private double filteredVelocity = 0;

  @Override
  public void robotInit() {
    setupShooterMotors();
    setupPidController();
    setupShuffleboardTab();
  }

  private void setupShooterMotors() {
    checkRevError(shooterMotor.enableVoltageCompensation(12.0));
    checkRevError(shooterMotorFollower.enableVoltageCompensation(12.0));
    checkRevError(shooterMotor.setIdleMode(IdleMode.kCoast));
    checkRevError(shooterMotorFollower.setIdleMode(IdleMode.kCoast));
    checkRevError(shooterMotorFollower.follow(shooterMotor, true));
    checkRevError(shooterMotor.setClosedLoopRampRate(0.005));

    motorEncoder = shooterMotor.getEncoder();
  }

  private void setupPidController() {
    motorPidController = shooterMotor.getPIDController();

    motorPidController.setP(p);
    motorPidController.setI(i);
    motorPidController.setD(d);
    motorPidController.setIZone(iz);
    motorPidController.setFF(ff);
    motorPidController.setOutputRange(minOutput, maxOutput);
  }

  private void setupShuffleboardTab() {
    DRIVE_SHUFFLEBOARD_TAB.add("PID", motorPidController);
    DRIVE_SHUFFLEBOARD_TAB.addNumber("Timer", () -> timer.get());
    DRIVE_SHUFFLEBOARD_TAB.addNumber("Shooter Motor Velocity", motorEncoder::getVelocity);
    DRIVE_SHUFFLEBOARD_TAB.addNumber("Filtered Velocity", () -> filteredVelocity);
    DRIVE_SHUFFLEBOARD_TAB.addNumber("Shooter Motor Output", shooterMotor::getAppliedOutput);
    DRIVE_SHUFFLEBOARD_TAB.addBoolean("AtVelocity?", this::atTargetVelocity);
  }

  @Override
  public void teleopPeriodic() {
    double targetRPM = targetRpmEntry.getDouble(targetRpm);

    if (driverController.getAButton() ) {
      if (timer.get() == 0) {
        timer.start();
      }
    } else {
      timer.reset();
    }

    if (targetRPM == 0) {
      shooterMotor.set(0);
    } else {
      motorPidController.setReference(targetRPM, CANSparkFlex.ControlType.kVelocity);
    }

    filteredVelocity = filter.calculate(motorEncoder.getVelocity());

    if (atTargetVelocity()) {
      timer.stop();
    }
  }

  private boolean atTargetVelocity() {
    return Math.abs(targetRpm - filteredVelocity) < 10;
  }

  
}