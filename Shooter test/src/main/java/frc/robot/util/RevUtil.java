// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj.DriverStation;

/** Add your docs here. */
public class RevUtil {
   
    public static void checkRevError(REVLibError error, String message) {
        if (error != REVLibError.kOk) {
            DriverStation.reportError(String.format("%s: %s", message, error.toString()), false);
        }
    }

    public static void setPeriodicFramePeriodLow(CANSparkMax motor) {
        checkRevError(motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0,  50), "Failed to set periodic status frame 0 rate");
        checkRevError(motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 500), "Failed to set periodic status frame 1 rate");
        checkRevError(motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 500), "Failed to set periodic status frame 2 rate");
        checkRevError(motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus3, 500), "Failed to set periodic status frame 3 rate");
    }
}
