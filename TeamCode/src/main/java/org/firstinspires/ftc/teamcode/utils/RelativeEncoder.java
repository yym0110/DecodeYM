package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RelativeEncoder {
    //private String name;
    public final AnalogInput encoder;

    private final double zeroOffset;
    private double currentAngle;
    private double prevAngle = 0;
    private double angleTraveled = 0;


    public RelativeEncoder(HardwareMap hardwareMap, String name, double offset) {
        encoder = hardwareMap.get(AnalogInput.class, name);

        zeroOffset = offset;

        currentAngle = Utils.headingClip(
                normalizeVoltage(encoder.getVoltage()) - zeroOffset
        );

        prevAngle = currentAngle;
    }

    public RelativeEncoder(HardwareMap hardwareMap, String name) {
        encoder = hardwareMap.get(AnalogInput.class, name);

        zeroOffset = 0;

        currentAngle = Utils.headingClip(
                normalizeVoltage(encoder.getVoltage()) - zeroOffset
        );

        prevAngle = currentAngle;
    }

    /**
     * Converts an analog voltage to an angle
     * @param v voltage [0,3.3]
     * @return angle [0,2PI]
     */
    public static double normalizeVoltage(double v) {
        return (v / 3.3) * 2.0 * Math.PI;
    }

    public void update() {

        currentAngle = Utils.headingClip(normalizeVoltage(encoder.getVoltage()) - zeroOffset);

        angleTraveled += Utils.headingClip(currentAngle - prevAngle);

        prevAngle = currentAngle;
    }

    public double getAngleTraveled() { return angleTraveled; }
    public void setAngleTraveled(double angle) { angleTraveled = angle; }
}
