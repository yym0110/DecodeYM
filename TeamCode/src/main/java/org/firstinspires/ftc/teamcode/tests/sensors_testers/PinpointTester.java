package org.firstinspires.ftc.teamcode.tests.sensors_testers;

import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_POSITION;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;

@TeleOp
public class PinpointTester extends LinearOpMode {
    public void runOpMode(){
        Robot robot = new Robot(hardwareMap);

        waitForStart();

        while(!isStopRequested()){
            Pose2d estimate = robot.drivetrain.localizers[2].getPoseEstimate();

            TelemetryUtil.packet.put("Pinpoint:: x", estimate.getX());
            TelemetryUtil.packet.put("Pinpoint:: y", estimate.getY());
            TelemetryUtil.packet.put("Pinpoint:: heading", estimate.getHeading());

            robot.update();
        }
    }
}
