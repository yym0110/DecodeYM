package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_LENGTH;
import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_WIDTH;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;

@Autonomous (name = "Red Tunnel Auto (Preload Only)", preselectTeleOp = "A. Teleop")
public class RedTunnelPreloadAuto extends LinearOpMode {
    Robot robot;
    long shooterTimer;

    @Override
    public void runOpMode(){
        robot = new Robot (hardwareMap);
        robot.intake.state = Intake.State.TEST;
        robot.setStopChecker(this::isStopRequested);

        robot.drivetrain.setPoseEstimate(new Pose2d(72 - ROBOT_LENGTH / 2, ROBOT_WIDTH / 2, Math.PI));

        while(opModeInInit()){
            robot.update();
        }

        TelemetryUtil.packet.put("Auto Stage", "Step 1");
        robot.drivetrain.goToPoint(new Pose2d(-8, 8, Math.PI * 2/3), 0.5);
        robot.shooter.setShooter(Shooter.State.MID);
        robot.shooter.setShooterBlocker(true);
        robot.update();
        // Wait for either the Drivetrain to be at point AND Shooter to reach the right velocity, OR for 5 seconds
        robot.waitWhile(() -> (robot.drivetrain.state != Drivetrain.State.WAIT || !robot.shooter.atVel()));

        TelemetryUtil.packet.put("Auto Stage", "Step 2");
        robot.shooter.setShooterBlocker(false);
        robot.intake.roller.setTargetPower(0.4);
        robot.intake.feed.setTargetPower(0.4);
        robot.update();
        // Wait for flywheel blocker to disengage
        robot.waitWhile(() -> !robot.shooter.flywheelBlocker.inPosition());

        TelemetryUtil.packet.put("Auto Stage", "Step 3");
        shooterTimer = System.currentTimeMillis();
        robot.intake.roller.setTargetPower(0.7);
        robot.intake.feed.setTargetPower(0.9);
        robot.update();
        // Upon disengage, shoot for 1.5 seconds
        robot.waitWhile(() -> System.currentTimeMillis() - shooterTimer <= 1500);

        TelemetryUtil.packet.put("Auto Stage", "Step 4");
        robot.shooter.setShooter(Shooter.State.OFF);
        robot.shooter.setShooterBlocker(true);
        robot.intake.roller.setTargetPower(0.0);
        robot.intake.feed.setTargetPower(0.0);
        robot.drivetrain.goToPoint(new Pose2d(33, ROBOT_WIDTH * 2/3, Math.PI / 2), 0.5);
        robot.update();
        // Disable all shooting mechanisms, move on to first row of intake balls
        // Wait for robot to reach point
        robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT);
    }
}
