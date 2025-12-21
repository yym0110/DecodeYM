package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.utils.ButtonToggle;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.RunMode;

@Config
@TeleOp(group = "Test")
public class ShooterTester extends LinearOpMode {
    public static boolean aimReq = false, shootReq = false, stopReq = false, intakeReq = false;

    public void runOpMode() {
        Globals.RUNMODE = RunMode.TESTER;
        Robot robot = new Robot(hardwareMap);

        while (opModeInInit()) {
            robot.update();
        }

        while (!isStopRequested()) {
            if (intakeReq) {
                robot.intake.reqIntake(true);
                intakeReq = false;
            }

            if (aimReq) {
                robot.shooter.reqAim(true);
                aimReq = false;
            }

            if (shootReq) {
                robot.shooter.reqShoot(true);
                shootReq = false;
            }

            if (stopReq) {
                robot.shooter.reqStop(true);
                stopReq = false;
            }

            robot.update();
        }
    }
}
