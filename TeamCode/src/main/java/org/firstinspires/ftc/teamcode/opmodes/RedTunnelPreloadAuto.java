package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_BACK_LENGTH;
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
@Autonomous(name = "RedTunnelPreloadAuto", group = "Auto")
public class RedTunnelPreloadAuto extends LinearOpMode {
    private Robot robot;
    public static long shootDuration = 1200;
    private long timer = System.currentTimeMillis();

    public static double heading = 2.71;

    public void runOpMode() {
        Globals.isRed = true;
        Globals.RUNMODE = RunMode.AUTO;
        robot = new Robot(hardwareMap);
        robot.setStopChecker(this::isStopRequested);
        robot.drivetrain.setPoseEstimate(new Pose2d(72 - ROBOT_BACK_LENGTH, 24 - ROBOT_WIDTH / 2, Math.PI));

        robot.shooter.state = Shooter.State.TEST;
        robot.shooter.setShooterBlocker(true);

        while (opModeInInit()) {
            robot.update();
            robot.sensors.light0G.setState(System.currentTimeMillis() % 500 < 250);
        }
        robot.sensors.light0G.setState(true);

        if (!isStopRequested()) LogUtil.init();
        LogUtil.drivePositionReset = true;

        robot.shooter.setShooter(Shooter.Dist.FAR);
        shoot(heading);
        intake(36, 54);
        shoot(heading);
        intake(12, 54);
        shoot(heading);
        intake(-12 , 50);
        shoot(heading);
        robot.shooter.setShooter(Shooter.Dist.OFF);
        robot.shooter.targetTurretAngle = 0.0;
        robot.drivetrain.goToPoint(new Pose2d(4, 45, Math.PI), 1.0);

        Globals.AUTO_ENDING_POSE = Globals.ROBOT_POSITION.clone();
        robot.waitWhile(() -> {
            Globals.AUTO_ENDING_POSE = Globals.ROBOT_POSITION.clone();
            return true;
        });
    }

    private void shoot(double heading) {
        robot.drivetrain.goToPoint(new Pose2d(53, 12, heading), 1.0);
        //robot.shooter.reqAim(true);

        robot.waitWhile(() -> {
            robot.shooter.turretTrackTarget();
            return robot.drivetrain.state != Drivetrain.State.WAIT || !robot.shooter.atVel() || Math.abs(robot.shooter.targetTurretAngle - robot.sensors.getTurretAngle()) > 3;
        });

        robot.shooter.setShooterBlocker(false);
        timer = System.currentTimeMillis();
        robot.intake.reqShoot(true);
        robot.waitWhile(() -> {
            robot.shooter.turretTrackTarget();
            return System.currentTimeMillis() - timer <= shootDuration;
        });

        robot.shooter.setShooterBlocker(true);
        robot.intake.reqOff(true);
    }

    private void intake(double x, double y) {
        robot.drivetrain.goToPoint(new Pose2d(x, 22, Math.toRadians(90)), 1.0);
        robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT);
        robot.intake.reqIntake(true);

        timer = System.currentTimeMillis();
        robot.drivetrain.goToPoint(new Pose2d(x, y, Math.toRadians(90)), 0.8);
        robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT && System.currentTimeMillis() - timer <= 3500);
    }
}
