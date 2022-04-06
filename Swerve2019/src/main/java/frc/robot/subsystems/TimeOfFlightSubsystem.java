// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TimeOfFlightSubsystem extends SubsystemBase {
  private TimeOfFlight m_timeOfFlight;
  
  /** Creates a new TimeOfFlight. */
  public TimeOfFlightSubsystem() {
    m_timeOfFlight = new TimeOfFlight(0x28); //40

    long firmwareVersion = m_timeOfFlight.getFirmwareVersion();
    m_timeOfFlight.setRangingMode(RangingMode.Medium,100);
    System.out.println("########## timeofflight "+m_timeOfFlight+" firmware = " + firmwareVersion);
    System.out.println("########## timeofflight light level = " + m_timeOfFlight.getAmbientLightLevel());
    System.out.println("########## timeofflight serial number = " + m_timeOfFlight.getSerialNumber());

    ShuffleboardTab tab = Shuffleboard.getTab("TOF");
    tab.addNumber("Range", this::getRange);
    tab.add("Time of flight", m_timeOfFlight);

  }

  public double getRange() {
    return m_timeOfFlight.getRange();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}
