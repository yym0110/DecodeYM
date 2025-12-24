package org.firstinspires.ftc.teamcode.tests.localization_testers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.utils.AngleUtil;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.vision.Vision;

@TeleOp
@Config
public class AprilTagLocalizationTest extends LinearOpMode {
    private Robot robot;
    private Vision vision;
    private LLResult result = null;

    public static double robotHeading = 0.0, turretHeading = 0.0;

    public void runOpMode() {
        vision = new Vision(hardwareMap);
        robot = new Robot(hardwareMap, vision);

        while (opModeInInit()) {
            robot.update();
            vision.update();
        }

        vision.setPipeline(0);
        vision.start();

        while (!isStopRequested()) {
            vision.update();
            robot.update();
            result = vision.getResult();

            if (result != null && result.isValid()) {
                double D = (Globals.tagHeight - Vision.cameraHeight) / Math.tan(Vision.cameraAngle + Math.toRadians(result.getTx()));
                double thetaLime = AngleUtil.clipAngle(robotHeading + turretHeading + Math.toRadians(result.getTy()));
                Pose2d tag = Globals.isRed ? Globals.redTag.clone() : Globals.blueTag.clone();

                Pose2d globalLimelightEstimate = new Pose2d(
                    tag.x - D * Math.cos(thetaLime) - 3.8582 * Math.sin(thetaLime),
                    tag.y - D * Math.sin(thetaLime) + 3.8582 * Math.cos(thetaLime),
                    Math.atan((D * Math.sin(thetaLime) + 3.8582 * Math.cos(thetaLime)) / (D * Math.cos(thetaLime) - 3.8582 * Math.sin(thetaLime))) - turretHeading
                );
                globalLimelightEstimate.heading += globalLimelightEstimate.x >= tag.x ? Math.PI : 0;

                TelemetryUtil.packet.put("Limelight Math x", globalLimelightEstimate.x);
                TelemetryUtil.packet.put("Limelight Math y", globalLimelightEstimate.y);
                TelemetryUtil.packet.put("Limelight Math heading", globalLimelightEstimate.heading);
            } else {
                TelemetryUtil.packet.put("Limelight Status: ", "Error");
            }
        }
    }
}
