package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class FeederSubsystem extends SubsystemBase {
    
    private PWMSparkMax m_motor = new PWMSparkMax(Constants.feederMotorPort);

    private DigitalInput m_sensor = new DigitalInput(Constants.eyeSensorChannel);

    public void initialize() {
    }

    boolean running = false;

    public void feed(double speed) {
        if ( !running ) {
          System.out.println("Starting feeder");
          running = true;
        }
        m_motor.set(speed);
    }

    public void stop() {
        System.out.println("Stopping feeder");
        running = false;
        m_motor.set(0.0);
    }

    public boolean isBallPresent() {
        return m_sensor.get();
    }
}
