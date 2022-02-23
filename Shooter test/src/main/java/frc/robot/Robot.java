/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
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
  private static final int MotorID1 = 16;
  private static final int MotorID2 = 17;
  private static final int FeederMotorID = 18;
  private static final int hopperMotorID = 12;
  private static final int gateMotorID = 2;
  private CANSparkMax m_motor;
  private CANSparkMax m_motortwo;
  private CANSparkMax m_feedermotor;
  private CANSparkMax m_hopperMotor;
  private CANSparkMax m_gateMotor;
  private SparkMaxPIDController m_pidController;
  private RelativeEncoder m_encoder;
  private RelativeEncoder m_encoder2;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, targetRPM;


  private static final int hood_deviceID = 19;
  private CANSparkMax m_motor_hood;
  private SparkMaxPIDController m_pidController_hood;

  private RelativeEncoder m_encoder_hood;
  public double kP_hood, kI_hood, kD_hood, kIz_hood, kFF_hood, kMaxOutput_hood, kMinOutput_hood;

  @Override
  public void robotInit() {
    m_stick = new XboxController(0);

    // initialize motor
    m_motor = new CANSparkMax(MotorID1, MotorType.kBrushless);
    m_motortwo = new CANSparkMax(MotorID2, MotorType.kBrushless);
    m_feedermotor = new CANSparkMax(FeederMotorID, MotorType.kBrushless);
    m_hopperMotor = new CANSparkMax(hopperMotorID, MotorType.kBrushless);
    m_gateMotor = new CANSparkMax(gateMotorID, MotorType.kBrushless);
    /**
     * The RestoreFactoryDefaults method can be used to reset the configuration parameters
     * in the SPARK MAX to their factory default state. If no argument is passed, these
     * parameters will not persist between power cycles
     */
    m_motor.restoreFactoryDefaults();
    m_motortwo.restoreFactoryDefaults();
    m_feedermotor.restoreFactoryDefaults();
    m_gateMotor.restoreFactoryDefaults();
    /**
     * In order to use PID functionality for a controller, a SparkMaxPIDController object
     * is constructed by calling the getPIDController() method on an existing
     * CANSparkMax object
     */
    m_pidController = m_motor.getPIDController();

    // Encoder object created to display position values
    m_encoder = m_motor.getEncoder();
    m_encoder2 = m_motortwo.getEncoder();

    m_motortwo.follow(m_motor, true);
    

    // PID coefficients
    kP = 6e-5; 
    kI = 0;
    kD = 0; 
    kIz = 0; 
    kFF = 0.00018;// 0.000015; 
    kMaxOutput = 1; 
    kMinOutput = -1;
    maxRPM = 5700;
    targetRPM = 0;

    // set PID coefficients
    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);
    m_pidController.setIZone(kIz);
    m_pidController.setFF(kFF);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);
    SmartDashboard.putNumber("targetRPM", targetRPM);

    initHood();
  }

  @Override
  public void teleopPeriodic() {

    m_feedermotor.set(m_stick.getRightY()/2);
    m_hopperMotor.set(-m_stick.getRightY());
    m_gateMotor.set(m_stick.getRightY());

    hoodPeriodic();
    // read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);
    double targetRPMb = SmartDashboard.getNumber("targetRPM", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP)) { m_pidController.setP(p); kP = p; }
    if((i != kI)) { m_pidController.setI(i); kI = i; }
    if((d != kD)) { m_pidController.setD(d); kD = d; }
    if((iz != kIz)) { m_pidController.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { m_pidController.setFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      m_pidController.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    }

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
    double setPoint = m_stick.getLeftY()*maxRPM;
    m_pidController.setReference(targetRPMb, CANSparkMax.ControlType.kVelocity);
    //m_motor.set(m_stick.getLeftY());

    
    SmartDashboard.putNumber("SetPoint", setPoint);
    SmartDashboard.putNumber("motor 1", m_encoder.getVelocity());
    SmartDashboard.putNumber("motor 2", m_encoder2.getVelocity());
  }



  public void initHood() {
    // initialize motor
    m_motor_hood = new CANSparkMax(hood_deviceID, MotorType.kBrushless);

    /**
     * The RestoreFactoryDefaults method can be used to reset the configuration parameters
     * in the SPARK MAX to their factory default state. If no argument is passed, these
     * parameters will not persist between power cycles
     */
    m_motor_hood.restoreFactoryDefaults();

    m_pidController_hood = m_motor_hood.getPIDController();


    // Encoder object created to display position values
    m_encoder_hood = m_motor_hood.getEncoder();

    m_encoder_hood.setPosition(0.0);
    
      // PID coefficients
      kP_hood = 0.1; 
      kI_hood = 1e-4;
      kD_hood = 1; 
      kIz_hood = 0; 
      kFF_hood = 0; 
      kMaxOutput_hood = 1; 
      kMinOutput_hood = -1;
  
      // set PID coefficients
      m_pidController_hood.setP(kP_hood);
      m_pidController_hood.setI(kI_hood);
      m_pidController_hood.setD(kD_hood);
      m_pidController_hood.setIZone(kIz_hood);
      m_pidController_hood.setFF(kFF_hood);
      m_pidController_hood.setOutputRange(kMinOutput_hood, kMaxOutput_hood);
  
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
    m_pidController_hood.setReference(rotations, CANSparkMax.ControlType.kPosition);
    
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
 // m_motor.set(m_stick.getLeftY() / 2);
//  
//  SmartDashboard.putNumber("SetPoint", setPoint);
//  SmartDashboard.putNumber("PositionVariable", m_encoder.getPosition());
//start at 0, max = 21
//negative for hood to extend
}

}