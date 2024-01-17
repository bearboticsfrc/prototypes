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
import edu.wpi.first.networktables.GenericEntry;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase;
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

  private final CANSparkFlex motor = new CANSparkFlex(4, MotorType.kBrushless);
  private final CANSparkFlex motorFollower = new CANSparkFlex(14, MotorType.kBrushless);
  private RelativeEncoder motorEncoder;
  private SparkPIDController motorPidController;

  private double targetRpm = 0;
  private double p = 0.0; // 0.001;
  private double i = 0.00001; // 0.00000007; 
  private double d = 0.000000008;
  private double iz = 0; // 400;   2000?
  private double ff = 0.0; // 0.000147;
  private double maxOutput = 1;
  private double minOutput = -1;

  private GenericEntry targetRpmEntry = DRIVE_SHUFFLEBOARD_TAB.add("targetRPM", targetRpm).getEntry();
  private GenericEntry pEntry = DRIVE_SHUFFLEBOARD_TAB.add("p", p).getEntry();
  private GenericEntry iEntry = DRIVE_SHUFFLEBOARD_TAB.add("i", i).getEntry();
  private GenericEntry dEntry = DRIVE_SHUFFLEBOARD_TAB.add("d", d).getEntry();
  private GenericEntry izEntry = DRIVE_SHUFFLEBOARD_TAB.add("iz", iz).getEntry();
  private GenericEntry ffEntry = DRIVE_SHUFFLEBOARD_TAB.add("ff", ff).getEntry();

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
    checkRevError(motor.restoreFactoryDefaults());
    checkRevError(motorFollower.restoreFactoryDefaults());
    checkRevError(motor.enableVoltageCompensation(12.0));
    motor.setInverted(false);
    checkRevError(motorFollower.enableVoltageCompensation(12.0));
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
    DRIVE_SHUFFLEBOARD_TAB.add("minOutput", minOutput);
    DRIVE_SHUFFLEBOARD_TAB.add("maxOutput", maxOutput);
    DRIVE_SHUFFLEBOARD_TAB.addNumber("Timer", timer::get);
    DRIVE_SHUFFLEBOARD_TAB.addNumber("Shooter Motor Velocity", motorEncoder::getVelocity);
    DRIVE_SHUFFLEBOARD_TAB.addNumber("Applied Output", motor::getAppliedOutput);
    DRIVE_SHUFFLEBOARD_TAB.addNumber("Applied Current", motor::getOutputCurrent);
    DRIVE_SHUFFLEBOARD_TAB.addNumber("Filtered Velocity", () -> filteredVelocity);
    DRIVE_SHUFFLEBOARD_TAB.addNumber("Shooter Motor Output", motor::getAppliedOutput);
    DRIVE_SHUFFLEBOARD_TAB.addBoolean("AtVelocity?", this::atTargetVelocity);
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

    if (driverController.getAButton() ) {
      if (timer.get() == 0) {
        timer.start();
      }
    } else {
      timer.reset();
    }

    if (targetRpm == 0) {
      motor.set(0);
    } else {
      checkRevError(motorPidController.setReference(targetRpm, CANSparkBase.ControlType.kVelocity));
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