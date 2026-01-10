package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drive.Path;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.LogUtil;
import org.firstinspires.ftc.teamcode.utils.Pose2d;

@Autonomous(name = "Red Goal")
public class RedGoal extends LinearOpMode {
    long delay;

    public void runOpMode() {
        Robot robot = new Robot(hardwareMap);
        LogUtil.init();

        // Location of turret center
        robot.drivetrain.setPoseEstimate(new Pose2d(-66, 42, 0));

        robot.setStopChecker(this::isStopRequested);

        while (opModeInInit()) {
            robot.sensors.update();
        }

        // Preload
        robot.shooter.reqAim(true);
        Path path = new Path(Globals.ROBOT_POSITION.clone(), Globals.getMidline())
                .addPoint(new Pose2d(-24, 24, Math.PI * 0.75))
                .setDecel(true);
        robot.drivetrain.setPath(path);
        robot.update();
        robot.waitWhile(() -> (robot.drivetrain.state != Drivetrain.State.WAIT || robot.shooter.state != Shooter.State.READY));

        robot.shooter.reqShoot(true);
        delay = System.currentTimeMillis();
        robot.update();
        robot.waitWhile(() -> (System.currentTimeMillis() - delay) < 1500);

        robot.shooter.reqStop(true);
        robot.update();

        // Goal side spikes + gate
        robot.intake.reqIntake(true);
        path = new Path(Globals.ROBOT_POSITION.clone(), Globals.getMidline())
                .setDecel(true)
                .addPoint(new Pose2d(-12, 24, Math.PI * 0.5))
                .addPoint(new Pose2d(-12, 42, Math.PI * 0.5));
        robot.drivetrain.setPath(path);
        robot.update();
        robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT);

        robot.intake.reqOff(true);
        path = new Path(Globals.ROBOT_POSITION.clone(), Globals.getMidline())
                .setDecel(true)
                .addPoint(new Pose2d(0, 50, 0));
        robot.drivetrain.setPath(path);
        robot.update();
        robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT);

        robot.shooter.reqAim(true);
        path = new Path(Globals.ROBOT_POSITION.clone(), Globals.getMidline())
                .addPoint(new Pose2d(-12, 12, Math.PI * 0.75));
        robot.drivetrain.setPath(path);
        robot.update();
        robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT || robot.shooter.state != Shooter.State.READY);

        robot.shooter.reqShoot(true);
        delay = System.currentTimeMillis();
        robot.update();
        robot.waitWhile(() -> (System.currentTimeMillis() - delay) < 1500);

        robot.shooter.reqStop(true);
        robot.update();

        // Middle spikes
        robot.intake.reqIntake(true);
        path = new Path(Globals.ROBOT_POSITION.clone(), Globals.getMidline())
                .setDecel(true)
                .addPoint(new Pose2d(12, 24, Math.PI * 0.5))
                .addPoint(new Pose2d(12, 42, Math.PI * 0.5));
        robot.drivetrain.setPath(path);
        robot.update();
        robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT);

        robot.intake.reqOff(true);
        robot.shooter.reqAim(true);
        path = new Path(Globals.ROBOT_POSITION.clone(), Globals.getMidline())
                .setDecel(true)
                .addPoint(new Pose2d(-12, 12, Math.PI * 0.75));
        robot.drivetrain.setPath(path);
        robot.update();
        robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT || robot.shooter.state != Shooter.State.READY);

        robot.shooter.reqShoot(true);
        delay = System.currentTimeMillis();
        robot.update();
        robot.waitWhile(() -> (System.currentTimeMillis() - delay) < 1500);

        robot.shooter.reqStop(true);
        robot.update();

        // Intake tunnel side spikes
        robot.intake.reqIntake(true);
        path = new Path(Globals.ROBOT_POSITION.clone(), Globals.getMidline())
                .setDecel(true)
                .addPoint(new Pose2d(36, 24, Math.PI * 0.5))
                .addPoint(new Pose2d(36, 42, Math.PI * 0.5));
        robot.drivetrain.setPath(path);
        robot.update();
        robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT);

        robot.intake.reqOff(true);
        robot.shooter.reqAim(true);
        path = new Path(Globals.ROBOT_POSITION.clone(), Globals.getMidline())
                .setDecel(true)
                .addPoint(new Pose2d(-12, 12, Math.PI * 0.75));
        robot.drivetrain.setPath(path);
        robot.update();
        robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT || robot.shooter.state != Shooter.State.READY);

        robot.shooter.reqShoot(true);
        delay = System.currentTimeMillis();
        robot.update();
        robot.waitWhile(() -> (System.currentTimeMillis() - delay) < 1500);

        robot.shooter.reqStop(true);
        robot.update();

        // Park
        path = new Path(Globals.ROBOT_POSITION.clone(), Globals.getMidline())
                .setDecel(true)
                .addPoint(new Pose2d(12, 24, Math.PI / 2));
        robot.drivetrain.setPath(path);
        robot.update();
        robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT);

        while(opModeIsActive()) { Globals.AUTO_ENDING_POSE = Globals.ROBOT_POSITION.clone(); }
    }
}
