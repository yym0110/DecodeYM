package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.utils.Globals.AUTO_ENDING_POSE;
import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_LENGTH;
import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_POSITION;
import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_WIDTH;

import com.google.ar.core.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;

//UPDATED FOR LM2

//can we combine steps 2 & 3 with the new bot? probably?
//add whatever turret auto aiming code and automatic shooter code
//we shoot the preloads, intake the first spike marks, open gate, and then continue to intake and shoot the spike marks
//ending pose is pointed towards small launch zone right next to gate for optimal teleop starting pose
//if we can shoot while moving, do that while shooting the first set of preloads

@Autonomous(name = "BlueGoalAuto", preselectTeleOp = "A. Teleop")
public class BlueGoalAuto extends LinearOpMode {
    Robot robot;
    long shooterTimer;

    @Override
    public void runOpMode(){
        robot = new Robot(hardwareMap);
        robot.intake.state = Intake.State.TEST;
        robot.setStopChecker(this::isStopRequested);

        robot.drivetrain.setPoseEstimate(new Pose2d(-72.0 + ROBOT_LENGTH / 2, -(48 - ROBOT_WIDTH / 2), 0));

        while(opModeInInit()){
            robot.update();
        }

        //move to the shooting position and start the shooting sequence
        TelemetryUtil.packet.put("Auto Stage", "Step 1 - Move to shooting position");
        robot.drivetrain.goToPoint(new Pose2d(-36, -36, Math.PI * 3/4), 0.5);
        //set shooter speed and hood angle
        robot.shooter.setShooter(Shooter.State.CLOSE);
        //make the blocker exist
        robot.shooter.setShooterBlocker(true);
        robot.update();
        robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT && !robot.shooter.atVel());

        //TODO add code for indexing

        TelemetryUtil.packet.put("Auto Stage", "Step 2 - Start shooting");
        //make it so the blocker isnt in the way
        robot.shooter.setShooterBlocker(false);
        //set the intake and feeding powers
        robot.intake.roller.setTargetPower(0.4);
        robot.intake.feed.setTargetPower(0.4);
        robot.update();
        //wait until the blocker gets into position
        robot.waitWhile(() -> !robot.shooter.flywheelBlocker.inPosition());

        TelemetryUtil.packet.put("Auto Stage", "Step 3 - Finish shooting");
        shooterTimer = System.currentTimeMillis();
        //set intake and feeding powers to be higher to get the rest of the balls
        robot.intake.roller.setTargetPower(0.7);
        robot.intake.feed.setTargetPower(0.9);
        robot.update();
        //shoot for 1.5 seconds and then stop
        robot.waitWhile(() -> System.currentTimeMillis() - shooterTimer <= 1500);

        TelemetryUtil.packet.put("Auto Stage", "Step 4 - move to first spike marks");
        robot.shooter.setShooter(Shooter.State.OFF);
        robot.shooter.setShooterBlocker(true);
        robot.intake.roller.setTargetPower(0.0);
        robot.intake.feed.setTargetPower(0.0);
        robot.drivetrain.goToPoint(new Pose2d(-9, -30, Math.PI / 2), 0.5);
        robot.update();
        robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT);

        TelemetryUtil.packet.put("Auto Stage", "Step 5 - grab first spike marks");
        robot.intake.roller.setTargetPower(0.9);
        robot.intake.feed.setTargetPower(0.4);
        robot.drivetrain.goToPoint(new Pose2d(-15, -60, Math.PI / 2), 0.5);
        robot.update();
        robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT);

        TelemetryUtil.packet.put("Auto Stage", "Step 6 - Open gate");
        robot.drivetrain.goToPoint(new Pose2d(0, -60, Math.PI / 2), 0.5);
        robot.update();
        robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT);

        TelemetryUtil.packet.put("Auto Stage", "Step 7 - move to shooting zone");
        robot.drivetrain.goToPoint(new Pose2d(-36, -36, Math.PI * 3/4), 0.5);
        robot.shooter.setShooter(Shooter.State.CLOSE);
        robot.shooter.setShooterBlocker(true);
        robot.update();
        robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT && !robot.shooter.atVel());

        TelemetryUtil.packet.put("Auto Stage", "Step 8 - Start shooting sequence");
        robot.shooter.setShooterBlocker(false);
        robot.intake.roller.setTargetPower(0.4);
        robot.intake.feed.setTargetPower(0.4);
        robot.update();
        robot.waitWhile(() -> !robot.shooter.flywheelBlocker.inPosition());

        TelemetryUtil.packet.put("Auto Stage", "Step 9 - Finish shooting sequence");
        shooterTimer = System.currentTimeMillis();
        robot.intake.roller.setTargetPower(0.7);
        robot.intake.feed.setTargetPower(0.9);
        robot.update();
        robot.waitWhile(() -> System.currentTimeMillis() - shooterTimer <= 1500);

        TelemetryUtil.packet.put("Auto State", "Step 10 - move to second spike marks");
        robot.shooter.setShooter(Shooter.State.OFF);
        robot.shooter.setShooterBlocker(true);
        robot.intake.roller.setTargetPower(0.0);
        robot.intake.feed.setTargetPower(0.0);
        robot.drivetrain.goToPoint(new Pose2d(12, -30, Math.PI / 2), 0.5);
        robot.update();
        robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT);

        TelemetryUtil.packet.put("Auto Stage", "Step 11 - grab second spike marks");
        robot.intake.roller.setTargetPower(0.9);
        robot.intake.feed.setTargetPower(0.4);
        robot.drivetrain.goToPoint(new Pose2d(12, -60, Math.PI / 2), 0.5);
        robot.update();
        robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT);

        TelemetryUtil.packet.put("Auto Stage", "Step 12 - move to shooting zone");
        robot.drivetrain.goToPoint(new Pose2d(-36, -36, Math.PI * 3/4), 0.5);
        robot.shooter.setShooter(Shooter.State.CLOSE);
        robot.shooter.setShooterBlocker(true);
        robot.update();
        robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT && !robot.shooter.atVel());

        TelemetryUtil.packet.put("Auto Stage", "Step 13 - Start shooting sequence");
        robot.shooter.setShooterBlocker(false);
        robot.intake.roller.setTargetPower(0.4);
        robot.intake.feed.setTargetPower(0.4);
        robot.update();
        robot.waitWhile(() -> !robot.shooter.flywheelBlocker.inPosition());

        TelemetryUtil.packet.put("Auto Stage", "Step 14 - Finish shooting sequence");
        shooterTimer = System.currentTimeMillis();
        robot.intake.roller.setTargetPower(0.7);
        robot.intake.feed.setTargetPower(0.9);
        robot.update();
        robot.waitWhile(() -> System.currentTimeMillis() - shooterTimer <= 1500);

        TelemetryUtil.packet.put("Auto State", "Step 15 - move to third spike marks");
        robot.shooter.setShooter(Shooter.State.OFF);
        robot.shooter.setShooterBlocker(true);
        robot.intake.roller.setTargetPower(0.0);
        robot.intake.feed.setTargetPower(0.0);
        robot.drivetrain.goToPoint(new Pose2d(36, -30, Math.PI / 2), 0.5);
        robot.update();
        robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT);

        TelemetryUtil.packet.put("Auto Stage", "Step 16 - grab third spike marks");
        robot.intake.roller.setTargetPower(0.9);
        robot.intake.feed.setTargetPower(0.4);
        robot.drivetrain.goToPoint(new Pose2d(36, -60, Math.PI / 2), 0.5);
        robot.update();
        robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT);

        TelemetryUtil.packet.put("Auto Stage", "Step 17 - move to shooting zone");
        robot.drivetrain.goToPoint(new Pose2d(-36, -36, Math.PI * 3/4), 0.5);
        robot.shooter.setShooter(Shooter.State.CLOSE);
        robot.shooter.setShooterBlocker(true);
        robot.update();
        robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT && !robot.shooter.atVel());

        TelemetryUtil.packet.put("Auto Stage", "Step 18 - Start shooting sequence");
        robot.shooter.setShooterBlocker(false);
        robot.intake.roller.setTargetPower(0.4);
        robot.intake.feed.setTargetPower(0.4);
        robot.update();
        robot.waitWhile(() -> !robot.shooter.flywheelBlocker.inPosition());

        TelemetryUtil.packet.put("Auto Stage", "Step 19 - Finish shooting sequence");
        shooterTimer = System.currentTimeMillis();
        robot.intake.roller.setTargetPower(0.7);
        robot.intake.feed.setTargetPower(0.9);
        robot.update();
        robot.waitWhile(() -> System.currentTimeMillis() - shooterTimer <= 1500);

        TelemetryUtil.packet.put("Auto Stage", "Step 20");
        robot.shooter.setShooter(Shooter.State.OFF);
        robot.shooter.setShooterBlocker(true);
        robot.intake.roller.setTargetPower(0.0);
        robot.intake.feed.setTargetPower(0.0);
        //right next to the gate pointed towards the small launch zone
        robot.drivetrain.goToPoint(new Pose2d(3, -48, 0), 0.5);
        robot.update();
        robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT);

        /*
        TelemetryUtil.packet.put("Auto Stage", "Step 11");
        robot.intake.roller.setTargetPower(0.9);
        robot.intake.feed.setTargetPower(0.4);
        robot.drivetrain.goToPoint(new Pose2d(9, 72.0 - ROBOT_LENGTH / 2.0, Math.PI / 2), 0.5);
        robot.update();
        robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT);

        TelemetryUtil.packet.put("Auto Stage", "Step 12");
        robot.drivetrain.goToPoint(new Pose2d(-6, 6, Math.PI * 3/4), 0.5);
        robot.shooter.setShooter(Shooter.State.MID);
        robot.shooter.setShooterBlocker(true);
        robot.update();
        robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT && !robot.shooter.atVel());

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
        robot.shooter.setShooter(Shooter.State.OFF);
        robot.shooter.setShooterBlocker(true);
        robot.intake.roller.setTargetPower(0.0);
        robot.intake.feed.setTargetPower(0.0);
        robot.drivetrain.goToPoint(new Pose2d(0, 12, 0), 0.5);
        robot.update();
        robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT);
        */

        AUTO_ENDING_POSE = ROBOT_POSITION;
        robot.waitWhile(this::isStopRequested);
    }
}
