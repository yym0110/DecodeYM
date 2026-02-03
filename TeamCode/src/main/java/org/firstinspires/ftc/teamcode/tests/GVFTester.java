package org.firstinspires.ftc.teamcode.tests;

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


@Autonomous(name = "GVF Tester", group = "Test")
public class GVFTester extends LinearOpMode {
    private Robot robot;


        public void runOpMode() {
            Globals.isRed = true;
            Globals.RUNMODE = RunMode.AUTO;
            robot = new Robot(hardwareMap);
            robot.setStopChecker(this::isStopRequested);
            robot.drivetrain.setPoseEstimate(new Pose2d(0, 0, 0));



            while (opModeInInit()) {
                robot.update();
                robot.sensors.light0G.setState(System.currentTimeMillis() % 500 < 250);
            }


            if (!isStopRequested()) LogUtil.init();
            LogUtil.drivePositionReset = true;

            Path path = new Path(Globals.ROBOT_POSITION.clone()).setDecel(true)
                    .addPoint(new Pose2d(0, 5, Math.toRadians(90)))
                    .addPoint(new Pose2d(-10, 5, 0))
                    .addPoint(new Pose2d(10, 10, Math.toRadians(90)))
                    .addPoint(new Pose2d(0, 0, 0));
            robot.drivetrain.setPath(path);
            robot.update();
            robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT);

            //robot.drivetrain.goToPoint(new Pose2d(-40, 40, 0), 1.0);
            //robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT);



            Globals.AUTO_ENDING_POSE = Globals.ROBOT_POSITION.clone();
            robot.waitWhile(() -> {
                Globals.AUTO_ENDING_POSE = Globals.ROBOT_POSITION.clone();
                return true;
            });
        }


    }


