/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
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
  private XboxController m_stick;
  private static final int deviceID = 19;
  private CANSparkMax m_motor;
  private SparkMaxPIDController m_pidController;

  private RelativeEncoder m_encoder;
  public double kP_hood, kI_hood, kD_hood, kIz_hood, kFF_hood, kMaxOutput_hood, kMinOutput_hood;

  @Override
  public void robotInit() {
    initHood();
  }

  public void initHood() {
    m_stick = new XboxController(0);

    // initialize motor
    m_motor = new CANSparkMax(deviceID, MotorType.kBrushless);

    /**
     * The RestoreFactoryDefaults method can be used to reset the configuration parameters
     * in the SPARK MAX to their factory default state. If no argument is passed, these
     * parameters will not persist between power cycles
     */
    m_motor.restoreFactoryDefaults();

    m_pidController = m_motor.getPIDController();


    // Encoder object created to display position values
    m_encoder = m_motor.getEncoder();

    m_encoder.setPosition(0.0);
    
      // PID coefficients
      kP_hood = 0.1; 
      kI_hood = 1e-4;
      kD_hood = 1; 
      kIz_hood = 0; 
      kFF_hood = 0; 
      kMaxOutput_hood = 1; 
      kMinOutput_hood = -1;
  
      // set PID coefficients
      m_pidController.setP(kP_hood);
      m_pidController.setI(kI_hood);
      m_pidController.setD(kD_hood);
      m_pidController.setIZone(kIz_hood);
      m_pidController.setFF(kFF_hood);
      m_pidController.setOutputRange(kMinOutput_hood, kMaxOutput_hood);
  
      // display PID coefficients on SmartDashboard
    //  SmartDashboard.putNumber("P Gain", kP_hood);
    //  SmartDashboard.putNumber("I Gain", kI_hood);
    //  SmartDashboard.putNumber("D Gain", kD_hood);
    //  SmartDashboard.putNumber("I Zone", kIz_hood);
    //  SmartDashboard.putNumber("Feed Forward", kFF_hood);
    //  SmartDashboard.putNumber("Max Output", kMaxOutput_hood);
      SmartDashboard.putNumber("Min Output", kMinOutput_hood);
      SmartDashboard.putNumber("Set Hood Rotations", 0);
   
  }

  private boolean hoodIsHome = false;
  @Override
  public void teleopPeriodic() {
    SmartDashboard.putNumber("Applied Output", m_motor.getAppliedOutput());
    SmartDashboard.putNumber("encoder", m_encoder.getVelocity());
    SmartDashboard.putNumber("Bus voltage", m_motor.getBusVoltage());
    SmartDashboard.putNumber("output current", m_motor.getOutputCurrent());

    if (m_stick.getAButton())
      m_motor.set(-0.08);
    else
      m_motor.set(0.0);

   if (m_stick.getBButton()) 
      homeHood();
  }

  private boolean first = true;

  private double initial_busVoltage = 0.0;
  public void homeHood() {
    if (hoodIsHome) return;
    if (initial_busVoltage == 0.0 )
       initial_busVoltage = m_motor.getBusVoltage();
    m_motor.set(-0.08);
    if ( m_motor.getOutputCurrent() > 10.0  && m_encoder.getVelocity() < 0.0001) {
      SmartDashboard.putBoolean("HOME",true);
      System.out.println("HOME!!!!!!");
      m_motor.set(0.0);
      m_motor.getEncoder().setPosition(0.0);
      hoodIsHome = true;
    }
    first = false;
  }

  public void hoodPeriodic() {

        // read PID coefficients from SmartDashboard
        //double p = SmartDashboard.getNumber("P Gain", 0);
        //double i = SmartDashboard.getNumber("I Gain", 0);
        //double d = SmartDashboard.getNumber("D Gain", 0);
        //double iz = SmartDashboard.getNumber("I Zone", 0);
        //double ff = SmartDashboard.getNumber("Feed Forward", 0);
        //double max = SmartDashboard.getNumber("Max Output", 0);
        //double min = SmartDashboard.getNumber("Min Output", 0);
        double rotations = SmartDashboard.getNumber("Set Hood Rotations", 0);
    
        // if PID coefficients on SmartDashboard have changed, write new values to controller
        //if((p != kP)) { m_pidController.setP(p); kP = p; }
        //if((i != kI)) { m_pidController.setI(i); kI = i; }
        //if((d != kD)) { m_pidController.setD(d); kD = d; }
        //if((iz != kIz)) { m_pidController.setIZone(iz); kIz = iz; }
        //if((ff != kFF)) { m_pidController.setFF(ff); kFF = ff; }
        //if((max != kMaxOutput) || (min != kMinOutput)) { 
        //  m_pidController.setOutputRange(min, max); 
        //  kMinOutput = min; kMaxOutput = max; 
        //}


    
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
        m_pidController.setReference(rotations, CANSparkMax.ControlType.kPosition);
        
       // SmartDashboard.putNumber("SetPoint", rotations);
       // SmartDashboard.putNumber("ProcessVariable", m_encoder.getPosition());
  
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
  //  double setPoint = m_stick.getLeftY()*maxRPM;
  //  m_motor.set(m_stick.getLeftY() / 2);
  //  
  //  SmartDashboard.putNumber("SetPoint", setPoint);
  //  SmartDashboard.putNumber("PositionVariable", m_encoder.getPosition());
    //start at 0, max = 21
    //negative for hood to extend
  }

  
}