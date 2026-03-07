package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_FORWARD_LENGTH;
import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_WIDTH;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.LogUtil;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.RunMode;

@Config
@Autonomous(name = "XX Red Tunnel Cycle Auto", group = "Auto", preselectTeleOp = "A. Teleop")
public class RedTunnelCycleAuto extends LinearOpMode {
    private Robot robot;
    public static long shootDuration = 700, intakeDuration = 1500, intakeMoveTimeout = 2000;

    public void runOpMode() {
        Globals.isRed = true;
        Globals.RUNMODE = RunMode.AUTO;
        robot = new Robot(hardwareMap);
        robot.setStopChecker(this::isStopRequested);
        robot.drivetrain.setPoseEstimate(new Pose2d(71 - ROBOT_WIDTH / 2, 23.75 - ROBOT_FORWARD_LENGTH, Math.PI / 2));

        robot.shooter.state = Shooter.State.TEST;
        robot.shooter.setShooterBlocker(true);
        robot.shooter.turretTrackInManual = true;

        while (opModeInInit()) {
            robot.update();
            robot.sensors.light0G.set(System.currentTimeMillis() % 500 < 100);
        }
        robot.sensors.light0G.set(false);

        if (!isStopRequested()) LogUtil.init();
        LogUtil.drivePositionReset = true;

        robot.shooter.setManual(false);

        robot.shooter.reqAim(true);

        shoot(Math.PI / 2, true);

        intakeSpikes(37,60);
        shoot(Math.PI / 2, false);

        for (int i = 0; i < 4; ++i) {
            intake(61, 60);
            shoot(Math.PI / 2, false);
        }

        robot.shooter.setManual(true);
        robot.shooter.setShooterBlocker(true);
        //robot.shooter.setShooter(Shooter.Dist.OFF);
        //robot.shooter.turret.setTargetAngle(0.0);
        robot.drivetrain.goToPoint(new Pose2d(63, 60, Math.PI / 2), 1.0);

        Globals.AUTO_ENDING_POSE = Globals.ROBOT_POSITION.clone();
        robot.waitWhile(() -> {
            Globals.AUTO_ENDING_POSE = Globals.ROBOT_POSITION.clone();
            return true;
        });
    }

    private void shoot(double heading, boolean firstShot) {
        //robot.shooter.reqAim(true);

        robot.drivetrain.goToPoint(new Pose2d(firstShot ? 63 : 60, 16, heading), 1);
        robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT || robot.shooter.state != Shooter.State.READY);
        robot.waitFor(firstShot ? 200 : 100);

        robot.shooter.reqShoot(true);
        robot.waitFor(shootDuration);
        robot.shooter.reqStop(true);
        robot.shooter.reqAim(true);
    }

    private void intakeSpikes(double x, double y) {
        // align
        robot.drivetrain.goToPoint(new Pose2d(x, 22, Math.PI / 2), 1, true);
        robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT);
        robot.intake.reqIntake(true);

        // intake
        robot.drivetrain.goToPoint(new Pose2d(x, y, Math.PI / 2), 1);
        robot.waitFor(intakeDuration);


        robot.intake.reqOff(true);
    }

    private void intake(double x, double y) {
        robot.drivetrain.goToPoint(new Pose2d(x, 22, Math.PI / 2), 1);
        robot.waitWhileWithTimeout(() -> robot.drivetrain.state != Drivetrain.State.WAIT, intakeMoveTimeout);
        robot.intake.reqIntake(true);

        robot.drivetrain.goToPoint(new Pose2d(x, y, Math.PI / 2), 1);
        robot.waitFor(intakeDuration);
        robot.intake.reqOff(true);

        robot.drivetrain.goToPoint(new Pose2d(x, 16, Math.PI / 2), 1, true);
        robot.waitWhileWithTimeout(() -> robot.drivetrain.state != Drivetrain.State.WAIT, intakeMoveTimeout);
    }
}
