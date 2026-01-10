package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drive.Path;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.Pose2d;

@Autonomous(name = "Red Goal")
public class RedGoal extends LinearOpMode {
    long delay;

    public void runOpMode() {
        Robot robot = new Robot(hardwareMap);

        // Location of turret center
        robot.drivetrain.setPoseEstimate(new Pose2d(-66, 42, 0));

        while (opModeInInit()) {
            robot.sensors.update();
        }

        robot.shooter.reqAim(true);
        Path path = new Path(new Pose2d(-66, 42, 0), Globals.getMidline())
                .addPoint(new Pose2d(-48, 24, Math.PI * 0.5))
                .addPoint(new Pose2d(-24, 24, Math.PI * 0.75))
                .setDecel(true);
        robot.drivetrain.setPath(path);
        robot.update();
        robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT && robot.shooter.state != Shooter.State.READY);

        robot.shooter.reqShoot(true);
        delay = System.currentTimeMillis();
        robot.update();
        robot.waitWhile(() -> (System.currentTimeMillis() - delay) < 1500);

        robot.shooter.reqStop(true);
        robot.update();

        robot.intake.reqIntake(true);
        path = new Path(new Pose2d(-12, 30, Math.PI * 0.5), Globals.getMidline())
                .setDecel(true)
                .addPoint(new Pose2d(-12, 37.5, Math.PI * 0.5));
        robot.drivetrain.setPath(path);
        robot.update();
        robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT);

        robot.intake.reqOff(true);
        path = new Path(new Pose2d(-4, 58, 0), Globals.getMidline())
                .setDecel(true);
        robot.drivetrain.setPath(path);
        robot.update();
        robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT);

        robot.shooter.reqAim(true);
        path = new Path(new Pose2d(-6, 6, Math.PI * 0.75), Globals.getMidline());
        robot.drivetrain.setPath(path);
        robot.update();
        robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT && robot.shooter.state != Shooter.State.READY);

        robot.shooter.reqShoot(true);
        delay = System.currentTimeMillis();
        robot.update();
        robot.waitWhile(() -> (System.currentTimeMillis() - delay) < 1500);

        robot.shooter.reqStop(true);
        robot.update();

        robot.intake.reqIntake(true);
        path = new Path(new Pose2d(12, 30, Math.PI * 0.5), Globals.getMidline())
                .setDecel(true)
                .addPoint(new Pose2d(12, 37.5, Math.PI * 0.5));
        robot.drivetrain.setPath(path);
        robot.update();
        robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT);

        robot.intake.reqIntake(true);
        robot.shooter.reqAim(true);
        path = new Path(new Pose2d(-6, 6, Math.PI * 0.75), Globals.getMidline());
        robot.drivetrain.setPath(path);
        robot.update();
        robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT && robot.shooter.state != Shooter.State.READY);

        robot.shooter.reqShoot(true);
        delay = System.currentTimeMillis();
        robot.update();
        robot.waitWhile(() -> (System.currentTimeMillis() - delay) < 1500);

        robot.shooter.reqStop(true);
        robot.update();

        robot.intake.reqIntake(true);
        path = new Path(new Pose2d(36, 30, Math.PI * 0.5), Globals.getMidline())
                .setDecel(true)
                .addPoint(new Pose2d(36, 37.5, Math.PI * 0.5));
        robot.drivetrain.setPath(path);
        robot.update();
        robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT);

        path = new Path(new Pose2d(12, 24, Math.PI / 2), Globals.getMidline());
        robot.drivetrain.setPath(path);
        robot.update();
        robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT);

        while(opModeIsActive()) { Globals.AUTO_ENDING_POSE = Globals.ROBOT_POSITION.clone(); }
    }
}
