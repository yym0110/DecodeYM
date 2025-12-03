package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.utils.Globals.AUTO_ENDING_POSE;
import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_LENGTH;
import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_POSITION;
import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_WIDTH;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;

@Autonomous(name = "Red Goal Auto", preselectTeleOp = "A.Teleop")
public class RedGoalAuto extends LinearOpMode {
    Robot robot;
    long shooterTimer;
    long stallTimer;

    @Override
    public void runOpMode(){
        robot = new Robot(hardwareMap);
        robot.intake.state = Intake.State.TEST;
        robot.setStopChecker(this::isStopRequested);

        robot.drivetrain.setPoseEstimate(new Pose2d(-72.0 + ROBOT_LENGTH / 2, 48 - ROBOT_WIDTH / 2, 0));

        while(opModeInInit()){
            robot.update();
        }

        TelemetryUtil.packet.put("Auto Stage", "Step 1");
        robot.drivetrain.goToPoint(new Pose2d(-36, 36, Math.PI * 3/4), 0.5);
        robot.shooter.setShooterBlocker(true);
        robot.update();
        robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT || !robot.shooter.atVel());

        TelemetryUtil.packet.put("Auto Stage", "Step 2");
        robot.shooter.setShooterBlocker(false);
        robot.intake.roller.setTargetPower(0.4);
        robot.intake.feed.setTargetPower(0.4);
        robot.update();
        robot.waitWhile(() -> !robot.shooter.flywheelBlocker.inPosition());

        TelemetryUtil.packet.put("Auto Stage", "Step 3");
        shooterTimer = System.currentTimeMillis();
        robot.intake.roller.setTargetPower(0.7);
        robot.intake.feed.setTargetPower(0.9);
        robot.update();
        robot.waitWhile(() -> System.currentTimeMillis() - shooterTimer <= 1500);

        TelemetryUtil.packet.put("Auto Stage", "Step 4");
        robot.shooter.setShooterBlocker(true);
        robot.intake.roller.setTargetPower(0.0);
        robot.intake.feed.setTargetPower(0.0);
        robot.drivetrain.goToPoint(new Pose2d(-18, 33, Math.PI / 2), 0.5);
        robot.update();
        robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT);

        TelemetryUtil.packet.put("Auto Stage", "Step 5");
        stallTimer = System.currentTimeMillis();
        robot.intake.roller.setTargetPower(0.9);
        robot.intake.feed.setTargetPower(0.4);
        robot.drivetrain.goToPoint(new Pose2d(-18, 60, Math.PI / 2), 0.2);
        robot.update();
        robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT && System.currentTimeMillis() - stallTimer <= 2000);

        TelemetryUtil.packet.put("Auto Stage", "Step 6");
        robot.drivetrain.goToPoint(new Pose2d(-36, 36, Math.PI * 3/4), 0.5);
        robot.shooter.setShooterBlocker(true);
        robot.update();
        robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT || !robot.shooter.atVel());

        TelemetryUtil.packet.put("Auto Stage", "Step 7");
        robot.shooter.setShooterBlocker(false);
        robot.intake.roller.setTargetPower(0.4);
        robot.intake.feed.setTargetPower(0.4);
        robot.update();
        robot.waitWhile(() -> !robot.shooter.flywheelBlocker.inPosition());

        TelemetryUtil.packet.put("Auto Stage", "Step 8");
        shooterTimer = System.currentTimeMillis();
        robot.intake.roller.setTargetPower(0.7);
        robot.intake.feed.setTargetPower(0.9);
        robot.update();
        robot.waitWhile(() -> System.currentTimeMillis() - shooterTimer <= 1500);

        TelemetryUtil.packet.put("Auto Stage", "Step 10");
        robot.shooter.setShooterBlocker(true);
        robot.intake.roller.setTargetPower(0.0);
        robot.intake.feed.setTargetPower(0.0);
        robot.drivetrain.goToPoint(new Pose2d(9, 30, Math.PI / 2), 0.5);
        robot.update();
        robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT);

        TelemetryUtil.packet.put("Auto Stage", "Step 11");
        stallTimer = System.currentTimeMillis();
        robot.intake.roller.setTargetPower(0.9);
        robot.intake.feed.setTargetPower(0.4);
        robot.drivetrain.goToPoint(new Pose2d(15, 72.0 - ROBOT_LENGTH / 2.0, Math.PI / 2), 0.5);
        robot.update();
        robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT && System.currentTimeMillis() - stallTimer <= 2000);

        TelemetryUtil.packet.put("Auto Stage", "Step 12");
        robot.drivetrain.goToPoint(new Pose2d(-8, 8, Math.PI * 2/3), 0.5);
        robot.shooter.setShooterBlocker(true);
        robot.update();
        robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT || !robot.shooter.atVel());

        TelemetryUtil.packet.put("Auto Stage", "Step 13");
        robot.shooter.setShooterBlocker(false);
        robot.intake.roller.setTargetPower(0.4);
        robot.intake.feed.setTargetPower(0.4);
        robot.update();
        robot.waitWhile(() -> !robot.shooter.flywheelBlocker.inPosition());

        TelemetryUtil.packet.put("Auto Stage", "Step 14");
        shooterTimer = System.currentTimeMillis();
        robot.intake.roller.setTargetPower(0.7);
        robot.intake.feed.setTargetPower(0.9);
        robot.update();
        robot.waitWhile(() -> System.currentTimeMillis() - shooterTimer <= 1500);

        TelemetryUtil.packet.put("Auto Stage", "Step 15");
        robot.shooter.setShooterBlocker(true);
        robot.intake.roller.setTargetPower(0.0);
        robot.intake.feed.setTargetPower(0.0);
        robot.drivetrain.goToPoint(new Pose2d(0, 24, 0), 0.5);
        robot.update();
        robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT);

        AUTO_ENDING_POSE = ROBOT_POSITION;
        robot.waitWhile(this::isStopRequested);
    }
}
