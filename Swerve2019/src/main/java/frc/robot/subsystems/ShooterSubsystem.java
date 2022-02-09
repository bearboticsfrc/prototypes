package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    private final BlinkinSubsystem m_blinkin;

    private boolean m_finishedShooting = false;

    public ShooterSubsystem(BlinkinSubsystem blinkinSubsystem) {
        m_blinkin = blinkinSubsystem;
    }

    public void shoot() {
        m_blinkin.set(BlinkinSubsystem.Color.STROBE_RED);
        m_finishedShooting = true;
    }

    public void prepare() {
        m_blinkin.set(BlinkinSubsystem.Color.GREEN);
        m_finishedShooting = false;
    }

    public void stop() {

    }

    public boolean finishedShooting() {
        return m_finishedShooting;
    }
}
