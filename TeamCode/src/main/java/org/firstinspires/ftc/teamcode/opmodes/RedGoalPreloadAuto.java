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
@Autonomous(name = "RedGoalPreloadAuto", group = "Auto")
public class RedGoalPreloadAuto extends LinearOpMode {
    private Robot robot;
    public static long shootDuration = 1000;
    private long timer = System.currentTimeMillis();

    public void runOpMode() {
        Globals.isRed = true;
        Globals.RUNMODE = RunMode.AUTO;
        robot = new Robot(hardwareMap);
        robot.setStopChecker(this::isStopRequested);
        robot.drivetrain.setPoseEstimate(new Pose2d(-72 + ROBOT_BACK_LENGTH, 24 + ROBOT_WIDTH / 2, 0));

        robot.shooter.state = Shooter.State.IDLE;
        robot.shooter.setShooterBlocker(true);

        while (opModeInInit()) {
            robot.update();
            robot.sensors.light0G.setState(System.currentTimeMillis() % 500 < 250);
        }
        robot.sensors.light0G.setState(true);

        if (!isStopRequested()) LogUtil.init();
        LogUtil.drivePositionReset = true;

        //robot.drivetrain.goToPoint(new Pose2d(-40, 40, 0), 1.0);
        //robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT);

        //robot.shooter.setShooter(Shooter.Dist.CLOSE);
        shoot(Math.PI / 4);
        intake(10, 56);
        open_gate();
        shoot(Math.PI);
        intake(-14, 50);
        shoot(Math.PI);
        intake(34, 56);
        shoot(Math.PI);
        //robot.shooter.setShooter(Shooter.Dist.OFF);
        robot.shooter.targetTurretAngle = 0.0;
        robot.drivetrain.goToPoint(new Pose2d(0, 45, Math.PI), 1.0);

        Globals.AUTO_ENDING_POSE = Globals.ROBOT_POSITION.clone();
        robot.waitWhile(() -> {
            Globals.AUTO_ENDING_POSE = Globals.ROBOT_POSITION.clone();
            return true;
        });
    }

    private void shoot(double heading) {
        robot.drivetrain.goToPoint(new Pose2d(-12, 12, heading), 1.0);
        robot.shooter.reqAim(true);

        robot.waitWhile(() ->  robot.drivetrain.state != Drivetrain.State.WAIT || robot.shooter.state != Shooter.State.READY);

        robot.shooter.reqShoot(true);
        timer = System.currentTimeMillis();
        robot.update();
        robot.waitWhile(() -> (System.currentTimeMillis() - timer) < shootDuration);

        robot.shooter.reqStop(true);
    }

    private void intake(double x, double y) {
        robot.drivetrain.goToPoint(new Pose2d(x, 22, Math.toRadians(90)), 1.0);
        robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT);
        robot.intake.reqIntake(true);

        timer = System.currentTimeMillis();
        robot.drivetrain.goToPoint(new Pose2d(x, y, Math.toRadians(90)), 0.6);
        robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT && System.currentTimeMillis() - timer <= 3500);

        robot.drivetrain.goToPoint(new Pose2d(x, 48, Math.toRadians(90)), 1.0, true);
        robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT);
        robot.intake.reqOff(true);
    }

    private void open_gate() {
        robot.drivetrain.goToPoint(new Pose2d(-4, 48, Math.PI/2), 1);
        robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT);

        timer = System.currentTimeMillis();
        robot.drivetrain.goToPoint(new Pose2d(-4, 53, Math.PI/2), 0.5);
        robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT && System.currentTimeMillis() - timer <= 1000);

        robot.drivetrain.goToPoint(new Pose2d(0, 48, Math.PI/2), 1, true);
        robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT);
    }
}
