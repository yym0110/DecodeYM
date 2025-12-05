package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_LENGTH;
import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_WIDTH;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;

@Autonomous(name = "ParkAuto", preselectTeleOp = "A. Teleop")
public class ParkAuto extends LinearOpMode {
    Robot robot;

    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap);
        robot.intake.state = Intake.State.TEST;
        robot.setStopChecker(this::isStopRequested);

        robot.drivetrain.setPoseEstimate(new Pose2d(72 - ROBOT_LENGTH / 2, -ROBOT_WIDTH / 2, Math.PI));

        while (opModeInInit()) {
            robot.update();
        }

        TelemetryUtil.packet.put("Auto Stage", "Step 1");
        robot.drivetrain.goToPoint(new Pose2d(26, -ROBOT_WIDTH / 2, Math.PI), 0.5);
        robot.update();
        robot.waitWhile(() -> (robot.drivetrain.state != Drivetrain.State.WAIT));
        TelemetryUtil.packet.put("Auto Stage", "Done");
        TelemetryUtil.sendTelemetry();
    }
}
