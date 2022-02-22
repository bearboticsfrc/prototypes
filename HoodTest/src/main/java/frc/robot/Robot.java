/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/**
 * This is a demo program showing the use of the RobotDrive class, specifically
 * it contains the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {
  private Joystick m_stick;
  private static final int deviceID = 1;
  private CANSparkMax m_motor;
  private RelativeEncoder m_encoder;
  public double maxRPM;

  @Override
  public void robotInit() {
    m_stick = new Joystick(0);

    // initialize motor
    m_motor = new CANSparkMax(deviceID, MotorType.kBrushless);

    /**
     * The RestoreFactoryDefaults method can be used to reset the configuration parameters
     * in the SPARK MAX to their factory default state. If no argument is passed, these
     * parameters will not persist between power cycles
     */
    m_motor.restoreFactoryDefaults();

    

    // Encoder object created to display position values
    m_encoder = m_motor.getEncoder();

    m_encoder.setPosition(0.0);

    // PID coefficients
    maxRPM = 5700;

    // set PID coefficients

    // display PID coefficients on SmartDashboard
   
  }

  @Override
  public void teleopPeriodic() {
    // read PID coefficients from SmartDashboard
    

    // if PID coefficients on SmartDashboard have changed, write new values to controller
   
    

    /**
     * PIDController objects are commanded to a set point using the 
     * SetReference() method.
     * 
     * The first parameter is the value of the set point, whose units vary
     * depending on the control type set in the second parameter.
     * 
     * The second parameter is the control type can be set to one of four 
     * parameters:
     *  com.revrobotics.CANSparkMax.ControlType.kDutyCycle
     *  com.revrobotics.CANSparkMax.ControlType.kPosition
     *  com.revrobotics.CANSparkMax.ControlType.kVelocity
     *  com.revrobotics.CANSparkMax.ControlType.kVoltage
     */
    double setPoint = m_stick.getY()*maxRPM;
    m_motor.set(m_stick.getY() / 10);
    
    SmartDashboard.putNumber("SetPoint", setPoint);
    SmartDashboard.putNumber("PositionVariable", m_encoder.getPosition());
  }
}