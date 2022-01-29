// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTableInstance;

/** Add your docs here. */
public class LimeLight {

    private boolean m_LimelightHasValidTarget = false;
    private boolean m_ledON = false;
    //private double m_LimelightDriveCommand = 0.0;
    public double m_LimelightSteerCommand = 0.0;
    public void Update_Limelight_Tracking()
    {
        //final double DRIVE_K = 0.26;
        final double DESIRED_TARGET_AREA = 13.0;
        final double MAX_DRIVE = 0.7;

        double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
        double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
        double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
        double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);

        //post to smart dashboard periodically
        SmartDashboard.putNumber("LimelightX", tx);
        SmartDashboard.putNumber("LimelightY", ty);
        SmartDashboard.putNumber("LimelightArea", ta);


        if (tv < 1.0)
        {
            m_LimelightHasValidTarget = false;
            //m_LimelightDriveCommand = 0.0;
            m_LimelightSteerCommand = 0.0;
            return;
        }

        m_LimelightHasValidTarget = true;

        double steer_cmd = tx;
        m_LimelightSteerCommand = steer_cmd;

        /* double drive_cmd = (DESIRED_TARGET_AREA - ta) * DRIVE_K;

        if (drive_cmd > MAX_DRIVE)
        {
            drive_cmd = MAX_DRIVE;
        }
        m_LimelightDriveCommand = drive_cmd; */
    }

    public void toggleLEDs() {
        if (m_ledON) {
            m_ledON = false;

            NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
        } else{
            m_ledON = true;

            NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
        }
    }
    public void setLEDs(boolean value) {
        if (value) {
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
            m_ledON = true;
        } else {
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
            m_ledON = false;
        }  
    }
    
    

}
