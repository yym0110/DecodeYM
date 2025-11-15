package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.RunMode;

@Config
@TeleOp(group = "Test")
public class ShooterTester extends LinearOpMode {
    public static double turretAngle = 0.0, hoodAngle = 0.0, flywheelVelocity = 0.0, flywheelPower = 0.0;
    public static boolean updateVelocity = false;

    public void runOpMode() {
        Globals.RUNMODE = RunMode.TESTER;
        Robot robot = new Robot(hardwareMap);
        Shooter shooter = robot.shooter;

        while (opModeInInit()) {
            robot.update();
        }

        while (!isStopRequested()) {
            robot.intake.roller.setTargetPower(1.0);
            robot.intake.feed.setTargetPower(0.7);

            shooter.setTurretAngle(turretAngle);
            shooter.setHoodAngle(hoodAngle);

            if (updateVelocity) {
                shooter.setTargetVelocity(flywheelVelocity);
                updateVelocity = false;
            }

            robot.update();
        }
    }
}
