package org.firstinspires.ftc.teamcode.tests.localization_testers;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.utils.ButtonToggle;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.RunMode;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;
import org.firstinspires.ftc.teamcode.vision.Vision;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

@TeleOp
public class LocalizationTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap, true);
        robot.shooter.state = Shooter.State.TEST;
        robot.shooter.turretTrackInManual = false;

        Globals.RUNMODE = RunMode.TESTER;

        waitForStart();

        robot.drivetrain.setPoseEstimate(new Pose2d(0, 0, 0));

        while (!isStopRequested()) {
            robot.drivetrain.drive(gamepad1);

            Pose2d pos = robot.drivetrain.getPoseEstimate();

            /*
            Log.i ("Localization Test - x", pos.x + "");
            Log.i ("Localization Test - y", pos.y + "");
            Log.i ("Localization Test - heading", pos.heading + "");
            Log.i ("Localization Test - odo 0", ((PriorityMotor) robot.hardwareQueue.getDevice("leftFront")).motor[0].getCurrentPosition() + "");
            Log.i ("Localization Test - odo 1", ((PriorityMotor) robot.hardwareQueue.getDevice("rightFront")).motor[0].getCurrentPosition() + "");
            Log.i ("Localization Test - odo 2", ((PriorityMotor) robot.hardwareQueue.getDevice("leftRear")).motor[0].getCurrentPosition() + "");
            */

            robot.update();
        }
    }
}