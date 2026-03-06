package org.firstinspires.ftc.teamcode.tests.sensors_testers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.RunMode;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;

@TeleOp
@Config
public class AnalogInputTest extends LinearOpMode {
    public static String encoderName = "turret_encoder";

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        Globals.RUNMODE = RunMode.TESTER;
        AnalogInput encoder = hardwareMap.get(AnalogInput.class, encoderName);

        waitForStart();

        while (opModeIsActive()) {

            TelemetryUtil.packet.put("encoderValue", encoder.getVoltage() / 3.3 * 360);
            telemetry.addData("encoderValue", encoder.getVoltage() / 3.3 * 360);
            telemetry.addData("quadatureEncoder", Math.toDegrees(robot.sensors.getTurretAngle()));
            telemetry.update();

            robot.sensors.update();
            // robot.update();
        }
    }
}
