/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel;
import static frc.robot.util.RevUtil.checkRevError;

import edu.wpi.first.math.filter.LinearFilter;

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
  private CANSparkMax m_motor;
  private CANSparkMax m_motortwo;
  private CANSparkMax m_feedermotor;
  private CANSparkMax m_hopperMotor;
  private SparkMaxPIDController m_pidController;

  private Encoder m_magEncoder;

  private PIDController m_roboRiopidController;
  private RelativeEncoder m_encoder;
  private RelativeEncoder m_encoder2;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, targetRPM;

  private Timer timer = new Timer();

  private LinearFilter filter = LinearFilter.movingAverage(20);

  private double filteredVelocity = 0.0;
  
  @Override
  public void robotInit() {
    m_stick = new XboxController(0);

    // initialize motor
    m_motor = new CANSparkMax(MotorID1, MotorType.kBrushless);
    m_motortwo = new CANSparkMax(MotorID2, MotorType.kBrushless);
    m_feedermotor = new CANSparkMax(FeederMotorID, MotorType.kBrushless);
    m_hopperMotor = new CANSparkMax(hopperMotorID, MotorType.kBrushless);

    m_magEncoder = new Encoder(5,6,true,EncodingType.k1X);
    m_magEncoder.setSamplesToAverage(60);

    m_magEncoder.setDistancePerPulse((1.0/1024.0)*60.0);
    /**
     * The RestoreFactoryDefaults method can be used to reset the configuration parameters
     * in the SPARK MAX to their factory default state. If no argument is passed, these
     * parameters will not persist between power cycles
     */
    //m_motor.restoreFactoryDefaults();
    //m_motortwo.restoreFactoryDefaults();
    //m_feedermotor.restoreFactoryDefaults();
    checkRevError(m_motor.enableVoltageCompensation(12.0), "Could not enableVoltageCompensation for shooter motor one.");
    checkRevError(m_motortwo.enableVoltageCompensation(12.0), "Could not enableVoltageCompensation for shooter motor two.");
    checkRevError(m_motor.setIdleMode(IdleMode.kCoast), "Could not set idle mode for shooter motor one.");
    checkRevError(m_motortwo.setIdleMode(IdleMode.kCoast), "Could not set idle mode for shooter motor two.");

    m_motor.setIdleMode(IdleMode.kCoast);
    m_motortwo.follow(m_motor, true);

    m_motor.setClosedLoopRampRate(0.005);

    checkRevError(m_motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 10), "Failed to set periodic status frame 0 rate");
    checkRevError(m_motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 10), "Failed to set periodic status frame 1 rate");
    checkRevError(m_motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 100), "Failed to set periodic status frame 2 rate");
    checkRevError(m_motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus3, 100), "Failed to set periodic status frame 3 rate");

    checkRevError(m_motortwo.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 10), "Failed to set periodic status frame 0 rate");
    checkRevError(m_motortwo.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 10), "Failed to set periodic status frame 1 rate");
    checkRevError(m_motortwo.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 100), "Failed to set periodic status frame 2 rate");
    checkRevError(m_motortwo.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus3, 100), "Failed to set periodic status frame 3 rate");


    /**
     * In order to use PID functionality for a controller, a SparkMaxPIDController object
     * is constructed by calling the getPIDController() method on an existing
     * CANSparkMax object
     */
    m_pidController = m_motor.getPIDController();


    // Encoder object created to display position values
    m_encoder = m_motor.getEncoder();
    m_encoder2 = m_motortwo.getEncoder();

    //private final double kP = 0.000055; //0.0001; //0.00004; // was 0.00006;
    //private final double kI = 0.0000005;
   

    // PID coefficients
    kP = 0.0001; 
    kI = 0.00000007;
    kD = 0.000000008; 
    kIz = 400; 
    kFF = 0.000168; // 0.0001745;// 0.000015; 
    kMaxOutput = 1; 
    kMinOutput = -1;
    maxRPM = 5700;
    targetRPM = 0;

    m_roboRiopidController = new PIDController(kP,kI,kD);

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
    SmartDashboard.putNumber("srxEncoder", m_magEncoder.getRate());


  }

  @Override
  public void teleopPeriodic() {

    m_feedermotor.set(m_stick.getRightY()/2);
    m_hopperMotor.set(-m_stick.getRightY());



    // read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);

    double targetRPM = 0.0;

    if ( m_stick.getAButton() ) {
      targetRPM = SmartDashboard.getNumber("targetRPM", 0);
      if ( timer.get() == 0.0 ) {
        timer.start();
      }
    } else {
      timer.reset();
    }

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
    //double setPoint = m_stick.getLeftY()*maxRPM;
    if ( targetRPM == 0.0 ) {
      m_motor.set(0.0);
    } else {
      m_pidController.setReference(targetRPM, CANSparkMax.ControlType.kVelocity);
    }

    //m_roboRiopidController.setSetpoint(targetRPM);
    //double value = m_roboRiopidController.calculate( m_magEncoder.getRate() );

    //value += kFF * targetRPMb;
    //SmartDashboard.putNumber("motor value", value);

   // m_motor.set(value);

    //m_motor.set(m_stick.getLeftY());

    filteredVelocity = filter.calculate(m_encoder.getVelocity());

    if ( Math.abs(targetRPM-filteredVelocity) < 10.0) {
      timer.stop();
    }
    
    //SmartDashboard.putNumber("SetPoint", setPoint);
    SmartDashboard.putNumber("timer", timer.get());
    SmartDashboard.putNumber("motor 1", m_encoder.getVelocity());
    SmartDashboard.putNumber("filtered vel", filteredVelocity);
    SmartDashboard.putNumber("motor1Output", m_motor.getAppliedOutput());
    SmartDashboard.putNumber("srxEncoder", m_magEncoder.getRate());
    SmartDashboard.putBoolean("AtVelocity", (Math.abs(targetRPM-filteredVelocity)<10.0));
  }



  
}