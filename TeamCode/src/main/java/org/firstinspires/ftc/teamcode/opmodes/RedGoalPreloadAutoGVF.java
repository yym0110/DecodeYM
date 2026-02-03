package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_BACK_LENGTH;
import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_WIDTH;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drive.Path;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.LogUtil;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.RunMode;

import java.util.ArrayList;

@Autonomous(name = "RedGoalPreloadAutoGVF", group = "Auto")
public class RedGoalPreloadAutoGVF extends LinearOpMode {
    private Robot robot;
    private double timer = System.currentTimeMillis();
    public static long shootDuration = 3000;
    public static long gateIntakeDuration = 3000;

    public void runOpMode() {
        Globals.isRed = true;
        Globals.RUNMODE = RunMode.AUTO;
        robot = new Robot(hardwareMap);
        robot.setStopChecker(this::isStopRequested);
        robot.drivetrain.setPoseEstimate(new Pose2d(-72 + ROBOT_BACK_LENGTH, 24 + ROBOT_WIDTH / 2, 0));

        robot.shooter.state = Shooter.State.TEST;
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

        robot.shooter.setShooter(Shooter.Dist.CLOSE);
        shoot(Math.PI / 4);
        intake(-12, 50);
        shoot(Math.PI);
        intake(12, 56);
        shoot(Math.PI);
        intake(36, 56);
        shoot(Math.PI);
        robot.shooter.setShooter(Shooter.Dist.OFF);
        robot.shooter.targetTurretAngle = 0.0;
        robot.drivetrain.goToPoint(new Pose2d(0, 45, Math.PI), 1.0);

        Globals.AUTO_ENDING_POSE = Globals.ROBOT_POSITION.clone();
        robot.waitWhile(() -> {
            Globals.AUTO_ENDING_POSE = Globals.ROBOT_POSITION.clone();
            return true;
        });
    }

    private void shoot(double heading) {

        robot.shooter.reqAim(true);

        Path path = new Path(Globals.ROBOT_POSITION.clone()).setDecel(true).addPoint(new Pose2d(-28, 28, heading));

        robot.drivetrain.setPath(path);
        robot.update();
        robot.waitWhile(() -> {
            robot.shooter.turretTrackTarget();
            return robot.drivetrain.state != Drivetrain.State.WAIT || robot.shooter.state != Shooter.State.READY || Math.abs(robot.shooter.targetTurretAngle - robot.sensors.getTurretAngle()) > 4;
        });

        robot.intake.reqOff(true);

        robot.shooter.reqShoot(true);
        timer = System.currentTimeMillis();
        robot.update();
        robot.waitWhile(() -> (System.currentTimeMillis() - timer) < shootDuration);

        robot.shooter.reqStop(true);
        robot.update();

    }

    private void intake(double x, double y) {
        ////path = new Path(Globals.ROBOT_POSITION.clone(), Globals.getMidline())
        //                .setDecel(true)
        //                .addPoint(new Pose2d(36, 30, Math.PI * 0.5))
        //                .addPoint(new Pose2d(36, 37.5, Math.PI * 0.5));
        //        robot.drivetrain.setPath(path);
        //        robot.update();
        //        robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT);

        robot.intake.reqIntake(true);

        Path path = new Path(Globals.ROBOT_POSITION.clone()).setDecel(true)
                .addPoint(new Pose2d(x, 22, Math.toRadians(90)))
                .addPoint(new Pose2d(x, y, Math.toRadians(90)))
                .addPoint(new Pose2d(x, 48, Math.toRadians(90)));
        robot.drivetrain.setPath(path);
        robot.update();
        robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT);

    }

    private void gate_intake() {
        robot.intake.reqIntake(true);

        Path path = new Path(Globals.ROBOT_POSITION.clone())
                .setDecel(true)
                .addPoint(new Pose2d(0, 45, Math.toRadians(90)))
                .addPoint(new Pose2d(0, 48, Math.toRadians(90)))
                .addPoint(new Pose2d(6, 50,Math.toRadians(135)));

        robot.drivetrain.setPath(path);
        robot.update();

        timer = System.currentTimeMillis();

        robot.waitWhile(() -> {
            robot.shooter.turretTrackTarget();
            return robot.drivetrain.state != Drivetrain.State.WAIT  || System.currentTimeMillis() - timer <= gateIntakeDuration;
        });


    }


}
