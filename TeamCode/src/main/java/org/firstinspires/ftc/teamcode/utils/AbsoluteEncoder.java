package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class AbsoluteEncoder{

    //private String name;
    private final AnalogInput encoder;
    private double encoder_pos;
    private double prev_angle = 0;
    private double angle_traveled = 0;
    private boolean first_update = true;

    public AbsoluteEncoder(String n, HardwareMap hardwareMap) {
        encoder = hardwareMap.get(AnalogInput.class, n);
    }

    private double normalizeVoltage(double v) {
        //outputs a number between 0 and 2pi radians
        //input is from 0 to 3.3
        return ( v / 3.3 ) * 2.0 * Math.PI;
    }



    public void updateEncoder() {
        encoder_pos = getEncoderPosition();
        if(first_update) {
            prev_angle = encoder_pos;
            first_update = false;
            return;
        }

        double delta = encoder_pos - prev_angle;

        //threshold
        if(delta > Math.PI) {
            delta -= 2.0 * Math.PI;
        } else if(delta < - Math.PI) {
            delta += 2.0 * Math.PI;
        }

        angle_traveled += delta;
        prev_angle = encoder_pos;
    }


    public double getEncoderPosition() {
        return normalizeVoltage(encoder.getVoltage());
    }

    public double getAngleTraveled() {
        return angle_traveled;
    }

}
