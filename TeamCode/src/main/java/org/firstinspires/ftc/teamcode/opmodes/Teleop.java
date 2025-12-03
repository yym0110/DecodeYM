package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.utils.Globals.AUTO_ENDING_POSE;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.utils.ButtonToggle;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.LogUtil;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.RunMode;

@Config
@TeleOp(name = "A. Teleop")
public class Teleop extends LinearOpMode {
    public static double feedPower = 0.6, idleFeedPower = 0.4, intakePower = 0.8;

    public void runOpMode() {
        Globals.RUNMODE = RunMode.TELEOP;
        Robot robot = new Robot(hardwareMap);
        robot.setStopChecker(this::isStopRequested);

        robot.drivetrain.setPoseEstimate(AUTO_ENDING_POSE);

        ButtonToggle lb1 = new ButtonToggle();
        ButtonToggle rb1 = new ButtonToggle();
        ButtonToggle a1 = new ButtonToggle();
        ButtonToggle b1 = new ButtonToggle();
        ButtonToggle x1 = new ButtonToggle();
        ButtonToggle y1 = new ButtonToggle();

        ButtonToggle a2 = new ButtonToggle();
        ButtonToggle b2 = new ButtonToggle();
        ButtonToggle x2 = new ButtonToggle();
        ButtonToggle y2 = new ButtonToggle();

        boolean intakeReversed = false;
        boolean intakeOn = false;
        boolean flywheelOn = false;
        boolean atSpeedRumble = false;
        boolean firstLoop = false;
//        Shooter.State state = Shooter.State.OFF;

        robot.intake.state = Intake.State.TEST;

        while (opModeInInit()) robot.update();

        if (!isStopRequested()) LogUtil.init();

        LogUtil.drivePositionReset = true;
        // robot.shooter.goalDetector.start();

        while (!isStopRequested()) {
            robot.update();

            if (lb1.isClicked(gamepad1.left_bumper)) {
                intakeReversed = false;
                intakeOn = !intakeOn;
            }
            if (a1.isClicked(gamepad1.a)) {
                intakeReversed = intakeOn && !intakeReversed;
                intakeOn = true;
            }
            robot.intake.roller.setTargetPowerSmooth(intakeOn ? (intakeReversed ? -intakePower : intakePower) : 0);

            if (b1.isHeld(gamepad1.b, 500) || b2.isHeld(gamepad2.b, 500)
            || y1.isHeld(gamepad1.y, 500) || y2.isHeld(gamepad2.y, 500)
            || x1.isHeld(gamepad1.x, 500) || x2.isHeld(gamepad2.x, 500)) { // Off
                flywheelOn = false;
//                robot.shooter.setShooter(Shooter.State.OFF);
//                state = Shooter.State.OFF;
            } else if (b1.isClicked(gamepad1.b) || b2.isClicked(gamepad2.b)) { // Close
                flywheelOn = true;
//                robot.shooter.setShooter(Shooter.State.CLOSE);
//                state = Shooter.State.CLOSE;
                atSpeedRumble = true;
                firstLoop = true;
            } else if (y1.isClicked(gamepad1.y) || y2.isClicked(gamepad2.y)) { // Middle
                flywheelOn = true;
//                robot.shooter.setShooter(Shooter.State.MID);
//                state = Shooter.State.MID;
                atSpeedRumble = true;
                firstLoop = true;
            } else if (x1.isClicked(gamepad1.x) || x2.isClicked(gamepad2.x)) { // Far
                flywheelOn = true;
//                robot.shooter.setShooter(Shooter.State.FAR);
//                state = Shooter.State.FAR;
                atSpeedRumble = true;
                firstLoop = true;
            }
            if (atSpeedRumble && firstLoop) {
                firstLoop = false;
            } else if (atSpeedRumble && robot.shooter.atVel()) {
                gamepad1.rumble(200);
                gamepad2.rumble(200);
                atSpeedRumble = false;
            }

            // activate feed / toggling flywheel blocker
            if ((gamepad1.right_bumper || gamepad2.right_bumper) /*&& state != Shooter.State.OFF*/) {
                robot.shooter.setShooterBlocker(false);
                robot.intake.feed.setTargetPower(robot.shooter.flywheelBlocker.inPosition() ? feedPower : 0);
            } else {
                robot.intake.feed.setTargetPower(intakeOn ? (intakeReversed ? -idleFeedPower : idleFeedPower) : 0);
                robot.shooter.setShooterBlocker(true);
            }

            // toggle alliance

            if (a2.isClicked(gamepad2.a)) {
                Globals.isRed = !Globals.isRed;
            }

            robot.drivetrain.drive(gamepad1);

            telemetry.addData("Alliance", Globals.isRed ? "Red" : "Blue");

            telemetry.addData("intakeOn", intakeOn);
            telemetry.addData("intakeReversed", intakeReversed);
            telemetry.addData("intakePower", robot.intake.roller.getPower());
            telemetry.addData("feedPower", robot.intake.feed.getPower());

            // telemetry.addData("shooter distance", state);
            telemetry.addData("flywheelOn", flywheelOn);
            telemetry.addData("flywheelAtVel", robot.shooter.atVel());
            telemetry.addData("flywheelError", robot.shooter.error);
            telemetry.addData("flywheel target velocity", robot.shooter.getTargetVelocity());


            telemetry.update();
        }
    }
}
