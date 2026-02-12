package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.utils.Globals.AUTO_ENDING_POSE;
import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_POSITION;
import static org.firstinspires.ftc.teamcode.utils.Globals.isRed;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.utils.ButtonToggle;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.LogUtil;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.RunMode;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.Utils;
import org.firstinspires.ftc.teamcode.vision.Vision;

import java.util.Locale;

@Config
@TeleOp(name = "A. Teleop")
public class Teleop extends LinearOpMode {

    public void runOpMode() {
        Globals.RUNMODE = RunMode.TELEOP;
        Robot robot = new Robot(hardwareMap, new Vision(hardwareMap));

        //robot.drivetrain.vision.start();
        robot.setStopChecker(this::isStopRequested);

        robot.shooter.state = Shooter.State.TEST;
        robot.shooter.turretTrackInManual = true;

        //robot.drivetrain.setPoseEstimate(AUTO_ENDING_POSE);

        robot.shooter.setShooterBlocker(true);

        ButtonToggle lb1 = new ButtonToggle();
        ButtonToggle rb1 = new ButtonToggle();
        ButtonToggle a1 = new ButtonToggle();
        ButtonToggle b1 = new ButtonToggle();
        ButtonToggle y1 = new ButtonToggle();
        ButtonToggle x1 = new ButtonToggle();
        ButtonToggle rt1 = new ButtonToggle();

        ButtonToggle a2 = new ButtonToggle();
        ButtonToggle b2 = new ButtonToggle();
        ButtonToggle x2 = new ButtonToggle();
        ButtonToggle y2 = new ButtonToggle();
        ButtonToggle back2 = new ButtonToggle();
        ButtonToggle h2 = new ButtonToggle();
        ButtonToggle v2 = new ButtonToggle();
        ButtonToggle lb2 = new ButtonToggle();
        ButtonToggle rb2 = new ButtonToggle();

        boolean intakeReversed = false;
        boolean intakeOn = false;
        boolean flywheelOn = false;
        boolean atSpeedRumble = false;
        boolean confirmation = true;
        final double triggerThresh = 0.2;

        while (opModeInInit()) {
            robot.sensors.update();
            TelemetryUtil.sendTelemetry();
        }

        if (!isStopRequested()) LogUtil.init();
        LogUtil.drivePositionReset = true;

        //robot.drivetrain.vision.start();

        while (!isStopRequested()) {
            robot.update();

            if (back2.isClicked(gamepad2.back)) {
                isRed = !isRed;
            }

            // INTAKE

            if (lb1.isClicked(gamepad1.left_bumper)) {
                intakeOn = !intakeOn;
                intakeReversed = false;
                if (intakeOn) {
                    robot.intake.reqIntake(true);
                } else {
                    robot.intake.reqOff(true);
                }
                robot.intake.setRollerDirection(false);
            }

            if (a1.isClicked(gamepad1.a && !gamepad1.start)) {
                intakeReversed = !intakeOn || !intakeReversed;
                intakeOn = true;
                robot.intake.reqIntake(true);
                robot.intake.setRollerDirection(intakeReversed);
            }

            // SHOOTER

            if (a2.isClicked(gamepad2.a)) {
                robot.shooter.setManual(true);
                robot.shooter.turretTrackInManual = false;
                gamepad1.rumble(500);
                gamepad2.rumble(500);
            }

            if (b2.isClicked(gamepad2.b && !gamepad2.start)) {
                robot.shooter.setManual(false);
                robot.shooter.setShooter(Shooter.Dist.OFF);
                gamepad1.rumble(500);
                gamepad2.rumble(500);
            }

            if (y2.isClicked(gamepad2.y)) {
                robot.shooter.turretTrackInManual = true;
                robot.shooter.setManual(true);
                gamepad1.rumble(500);
                gamepad2.rumble(500);
            }

            if (x2.isClicked(gamepad2.x)) {
                Sensors.resetTurretAngleEncoder = true;
                gamepad1.rumble(50);
                gamepad2.rumble(50);
            }

            if (robot.shooter.state == Shooter.State.TEST) {
                rb1.isClicked(gamepad1.right_bumper);

                if (rt1.isClicked(gamepad1.right_trigger > triggerThresh) || b1.isHeld(gamepad1.b, 500)
                        || y1.isHeld(gamepad1.y, 500)
                        || x1.isHeld(gamepad1.x, 500)) { // Off
                    flywheelOn = false;
                    robot.shooter.setShooter(Shooter.Dist.OFF);
                    atSpeedRumble = false;
                } else if (b1.isClicked(gamepad1.b)) { // Close
                    flywheelOn = true;
                    robot.shooter.setShooter(Shooter.Dist.CLOSE);
                    atSpeedRumble = true;
                    confirmation = true;
                } else if (y1.isClicked(gamepad1.y)) { // Middle
                    flywheelOn = true;
                    robot.shooter.setShooter(Shooter.Dist.MID);
                    atSpeedRumble = true;
                    confirmation = true;
                } else if (x1.isClicked(gamepad1.x)) { // Far
                    flywheelOn = true;
                    robot.shooter.setShooter(Shooter.Dist.FAR);
                    atSpeedRumble = true;
                    confirmation = true;
                }

                if (atSpeedRumble && confirmation) {
                    confirmation = false;
                } else if (atSpeedRumble && robot.shooter.atVel()) {
                    gamepad1.rumble(100);
                    gamepad2.rumble(100);
                    atSpeedRumble = false;
                }
                if (robot.shooter.atVel()) {
                    robot.sensors.light0G.set(true);
                    robot.sensors.light0P.set(true);
                }

                if (gamepad1.right_bumper) {
                    rb1.isReleased(gamepad1.right_bumper);
                    robot.shooter.setShooterBlocker(false);
                    robot.intake.reqShoot(true);
                } else if (rb1.isReleased(gamepad1.right_bumper)) {
                    robot.shooter.setShooterBlocker(true);
                    intakeOn = false;
                    robot.intake.reqOff(true);
                }
            } else {
                x1.isClicked(gamepad1.x);

                if (y1.isClicked(gamepad1.y)) {
                    robot.shooter.reqAim(true);
                }

                if (rb1.isClicked(gamepad1.right_bumper)) {
                    if (robot.shooter.state == Shooter.State.READY) robot.shooter.reqShoot(true);
                    else if (robot.shooter.state == Shooter.State.IDLE) robot.shooter.reqAim(true);
                }

                if (robot.shooter.state == Shooter.State.READY) {
                    if (confirmation) {
                        gamepad1.rumble(150);
                        gamepad2.rumble(150);
                        confirmation = false;
                    }
                } else {
                    confirmation = true;
                }

                if (rt1.isClicked(gamepad1.right_trigger > triggerThresh) || b1.isClicked(gamepad1.b)) {
                    robot.shooter.reqStop(true);
                }
            }

            if (h2.isClicked(gamepad2.dpad_left || gamepad2.dpad_right)) { // localize to left/right edge (unchanged x, auto y, auto h)
                double h = Utils.headingClip(ROBOT_POSITION.heading);
                if (h < Math.toRadians(-135)) h = -Math.PI;
                else if (h < Math.toRadians(-45)) h = -Math.PI / 2;
                else if (h > Math.toRadians(135)) h = Math.PI;
                else if (h > Math.toRadians(45)) h = Math.PI / 2;
                else h = 0;
                robot.drivetrain.setPoseEstimate(new Pose2d(ROBOT_POSITION.x, (ROBOT_POSITION.y > 0 ? 1 : -1) * (71 - 6.5), h));
                gamepad1.rumble(1000);
                gamepad2.rumble(1000);
            }

            if (v2.isClicked(gamepad2.dpad_up || gamepad2.dpad_down)) { // localize to top/bottom edge (auto x, unchanged y, auto h)
                double h = Utils.headingClip(ROBOT_POSITION.heading);
                if (h < Math.toRadians(-135)) h = -Math.PI;
                else if (h < Math.toRadians(-45)) h = -Math.PI / 2;
                else if (h > Math.toRadians(135)) h = Math.PI;
                else if (h > Math.toRadians(45)) h = Math.PI / 2;
                else h = 0;
                robot.drivetrain.setPoseEstimate(new Pose2d((ROBOT_POSITION.x > 0 ? 1 : -1) * (71 - 6.5), ROBOT_POSITION.y, h));
                gamepad1.rumble(800);
                gamepad2.rumble(800);
            }

            if (lb2.isClicked(gamepad2.left_bumper)) LogUtil.event.add("ballMiss");
            else if (rb2.isClicked(gamepad2.right_bumper)) LogUtil.event.add("ballHit");

            robot.drivetrain.drive(gamepad1);

            telemetry.addData("Alliance", Globals.isRed ? "Red" : "Blue");
            telemetry.addData("intakeReversed", intakeReversed);
            telemetry.addData("intakePower", robot.intake.roller.getPower());
            telemetry.addData("shooter state", robot.shooter.state.toString());
            telemetry.addData("turretTrackInManual", robot.shooter.turretTrackInManual);
            telemetry.addData("flywheelOn", flywheelOn);
            telemetry.addData("flywheelAtVel", robot.shooter.atVel());
            telemetry.addData("turretInPosition", Math.abs(robot.shooter.targetTurretAngle - robot.sensors.getTurretAngle()) <= Math.toRadians(2.0) ? "yes" : "aw no its not happy yet");
            telemetry.addData("Robot position (deg)", String.format(Locale.US, "(%.2f, %.2f, %.2f)", ROBOT_POSITION.x, ROBOT_POSITION.y, Math.toDegrees(ROBOT_POSITION.heading)));

            telemetry.update();
        }

        Globals.AUTO_ENDING_POSE = Globals.ROBOT_POSITION.clone();
        robot.waitWhile(() -> {
            Globals.AUTO_ENDING_POSE = Globals.ROBOT_POSITION.clone();
            return true;
        });
    }
}
