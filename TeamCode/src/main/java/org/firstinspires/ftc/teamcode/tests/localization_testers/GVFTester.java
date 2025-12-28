package org.firstinspires.ftc.teamcode.tests.localization_testers;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.drive.Path;
import org.firstinspires.ftc.teamcode.subsystems.drive.RepulsionPoint;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.utils.Pose2d;

import java.util.ArrayList;

@Autonomous
public class GVFTester extends LinearOpMode{
    private Robot robot;

    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);
        robot.shooter.state = Shooter.State.TEST;

        robot.drivetrain.setPoseEstimate(new Pose2d (0, 0, 0));

        ArrayList<RepulsionPoint> repel = new ArrayList<>();

        Path testPath = new Path(new Pose2d(0, 0, 0), repel)
                .addPoint(new Pose2d(24, 0, Math.PI / 2))
                .addPoint(new Pose2d(48, 24, Math.PI / 2));

        waitForStart();

        while(opModeInInit()){
            robot.update();
        }

        robot.drivetrain.setPath(testPath);

        while(!isStopRequested()){
            robot.update();
        }
    }
}
