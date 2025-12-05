package org.firstinspires.ftc.teamcode.sensors;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.DashboardUtil;
import org.firstinspires.ftc.teamcode.utils.LogUtil;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.Vector2;

@Config
public class Sensors {
    private final Robot robot;
    public double loopTime;

    private int[] odoWheelPositions = {0, 0, 0};

    // 426 mm (16.772 in) is belt circumference
    // start: 42638
    // end: 43062
    private double flywheelVelocity, ticksToInches = 16.772 / (43062 - 42638), flywheelVelo2, flywheelLastPos = 0;

    private double voltage;
    private final double voltageUpdateTime = 5000;
    private long lastVoltageUpdatedTime = System.currentTimeMillis();

    public Sensors(Robot robot) {
        this.robot = robot;
        voltage = robot.hardwareMap.voltageSensor.iterator().next().getVoltage();
    }

    public void update() {
        odoWheelPositions[0] = robot.drivetrain.rightFront.motor[0].getCurrentPosition(); // left
        odoWheelPositions[1] = robot.drivetrain.leftRear.motor[0].getCurrentPosition(); // right
        odoWheelPositions[2] = robot.drivetrain.leftFront.motor[0].getCurrentPosition(); // back

        double flywheelPos = robot.drivetrain.rightRear.motor[0].getCurrentPosition();
        flywheelVelocity = robot.drivetrain.rightRear.getVelocity() * ticksToInches;
        flywheelVelo2 = (flywheelPos - flywheelLastPos) / loopTime * ticksToInches;
        flywheelLastPos = flywheelPos;

        if (System.currentTimeMillis() - lastVoltageUpdatedTime > voltageUpdateTime) {
            voltage = robot.hardwareMap.voltageSensor.iterator().next().getVoltage();
            lastVoltageUpdatedTime = System.currentTimeMillis() ;
        }

        updateTelemetry();
    }

    // Odometry
    public int[] getOdometry() {return odoWheelPositions;}

    public double getFlywheelVelocity () { return flywheelVelocity; }

    public double getVoltage() {
        return voltage;
    }

    private void updateTelemetry() {
        TelemetryUtil.packet.put("Voltage", voltage);
        TelemetryUtil.packet.put("Shooter : Flywheel velocity", flywheelVelocity);
        TelemetryUtil.packet.put("Shooter : Flywheel velocity 2", flywheelVelo2);
    }
}
