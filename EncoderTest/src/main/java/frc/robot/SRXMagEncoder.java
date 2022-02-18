package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class SRXMagEncoder {

    private DutyCycleEncoder dutyCycleEncoder;

    private double m_offset = 0.0;

    private DutyCycle m_dutyCycle;

    private int m_pwmInputPort;

    public SRXMagEncoder(int pwmInputPort, double offset) {
        this.m_offset = offset;
        this.m_pwmInputPort = pwmInputPort;
        DigitalInput digitalInput = new DigitalInput(pwmInputPort);
        m_dutyCycle = new DutyCycle(digitalInput);
        dutyCycleEncoder = new DutyCycleEncoder(m_dutyCycle);
        dutyCycleEncoder.setDistancePerRotation(360.0);
        dutyCycleEncoder.setDutyCycleRange(0.0, 1.0);
    }

    public double getAbsolutePosition2() {
        double distance = getAbsolutePositionDutyCycle() - m_offset;
        double position = distance %= 360.0;
        if (position < 0.0)
            position += 360.0;

        return position;
    }

    public double getAbsolutePosition() {
        double distance = dutyCycleEncoder.getDistance() - m_offset;
        double position = distance %= 360.0;
        if (position < 0.0)
            position += 360.0;

       if (m_pwmInputPort == 0)
            System.out.println("position=" + position + " , other=" + getAbsolutePosition2());

        return position;
    }

    boolean m_absoluteSensorGood = true;
    int m_alignmentOffset = 0;
    // Calulate absolute turning position in the range [-2048, +2048), from the raw
    // absolute PWM encoder data. No value is returned in the event the absolute
    // position sensor itself is not returning valid data. The alignment offset is
    // optionally used to establish the zero position.

    // This is a low-level routine, meant to be used only by the version of
    // GetAbsolutePosition() with no arguments (below) or, from test mode. It does
    // not use standardized units and leaks knowledge of the sensor output range.
    public int getAbsolutePosition(int frequency, double output, boolean applyOffset) {
        // SRX MAG ENCODER chip seems to be AS5045B; from the datasheet, position
        // is given by:
        // ((t_on * 4098) / (t_on + t_off)) - 1;
        // if this result is 4096, set it to 4095.
        // This computation results in a position in the range [0, 4096).

        // If the frequency isn't within the range specified in the data sheet,
        // return an error. This range is [220, 268], with a nominal value of 244.
        // A tolerance of 12 (~5% of nominal) is provided for any measurment error.
        m_absoluteSensorGood = frequency >= 208 && frequency <= 280;

        if (!m_absoluteSensorGood)
            return 0;

        // GetOutput() is (t_on / (t_on + t_off)); this is all that we have; note
        // that this is sampled at a high frequency and the sampling window may not
        // be perfectly aligned with the signal, although the frequency should be
        // employed to ensure the sampling window is sized to a multiple of the
        // period of the signal. So, t_on and t_off have a different scale in this
        // context, which is OK. We are using the duty cycle (ratio) only.

        // Multiply by 4098 and subtract 1; should yield 0 - 4094, and also 4096.
        // Conditionals cover special case of 4096 and enforce the specified range.
        int position = (int) (output * 4098.0) - 1;
        position = MathUtil.clamp(position, 0, 4095);

        // Now, shift the range from [0, 4096) to [-2048, +2048).
        //position -= 2048;

        // There is a mechanical offset reflecting the alignment of the magnet with
        // repect to the axis of rotation of the wheel (and also any variance in
        // the alignment of the sensor). Add this in here to reference position to
        // the desired zero position.
        // position += m_alignmentOffset;
        //position += m_offset;
        if (position > 2047) {
            position -= 4096;
        }
        if (position < -2048) {
            position += 4096;
        }

        return position;
    }

    // As above; obtain raw data from DutyCycle object.
    public double getAbsolutePositionDutyCycle() {
        int position = getAbsolutePosition(m_dutyCycle.getFrequency(),
                m_dutyCycle.getOutput(),
                true);

        double value = (position / 4096.0) * 360.0;

        return value;
    }
}
