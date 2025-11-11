package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.LogUtil;
import org.firstinspires.ftc.teamcode.utils.RunMode;

@TeleOp(name = "A. Teleop")
public class Teleop extends LinearOpMode {
    public void runOpMode() {
        Globals.RUNMODE = RunMode.TELEOP;
        Robot robot = new Robot(hardwareMap);
        robot.setStopChecker(this::isStopRequested);

        robot.intake.state = Intake.State.TEST;

        while (opModeInInit()) {
            robot.update();
        }

        if (!isStopRequested()) LogUtil.init();
        LogUtil.drivePositionReset = true;

        while (!isStopRequested()) {
            robot.update();

            robot.shooter.setShooterPower(gamepad1.right_trigger * 0.8);
            if (gamepad1.right_bumper) robot.intake.feed.setTargetPower(0.5);
            else robot.intake.feed.setTargetPower(0);
            if (gamepad1.left_bumper) robot.intake.roller.setTargetPower(0.75);
            else robot.intake.roller.setTargetPower(0);
            robot.drivetrain.drive(gamepad1);
        }
    }
}
