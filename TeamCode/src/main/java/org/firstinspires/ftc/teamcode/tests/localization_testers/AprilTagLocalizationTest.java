package org.firstinspires.ftc.teamcode.tests.localization_testers;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.utils.AngleUtil;
import org.firstinspires.ftc.teamcode.utils.DashboardUtil;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.vision.Vision;

@TeleOp(group = "Test")
@Config

public class AprilTagLocalizationTest extends LinearOpMode {
    private Vision vision;
    private LLResult result = null;

    public static double robotHeading = 0.0;

    @SuppressLint("DefaultLocale")
    public void runOpMode() {
        TelemetryUtil.setup();

        vision = new Vision(hardwareMap);

        while (opModeInInit()) {
            vision.update();
        }

        vision.setPipeline(0);
        vision.start();

        while (!isStopRequested()) {
            vision.update();
            result = vision.getResult();

            if (result != null && result.isValid()) {
                double D = (Globals.tagHeight - Vision.cameraHeight) / Math.tan(Math.toRadians(0.97 - 0.729 * result.getTx() + 9.37 * 0.001 * result.getTx() * result.getTx()));
                double ty = Math.toRadians(2.88 + 0.249 * result.getTy() + 0.0325 * result.getTy() * result.getTy());

                //monkey numbers cuz why not
                double thetaLime = AngleUtil.clipAngle(robotHeading - ty) + Math.PI / 4;
                Pose2d tag = Globals.isRed ? Globals.redTag.clone() : Globals.blueTag.clone();

                Pose2d estimatedLLPos = new Pose2d(
                    tag.x + D * Math.cos(thetaLime),
                    tag.y - D * Math.sin(thetaLime),
                        Math.atan2(tag.y - D * Math.sin(thetaLime), tag.x - D * Math.cos(thetaLime) + thetaLime) // radians from -pi to pi
                );

                 Pose2d globalLLEstimate = new Pose2d(
                         estimatedLLPos.x - 6.4 * Math.cos(estimatedLLPos.heading) + 5.5 * Math.sin(estimatedLLPos.heading),
                         estimatedLLPos.y - 6.4 * Math.sin(estimatedLLPos.heading) + 5.5 * Math.cos(estimatedLLPos.heading),
                         estimatedLLPos.heading
                );

                globalLLEstimate.heading += globalLLEstimate.x >= tag.x ? Math.PI : 0;

                TelemetryUtil.packet.put("LL D", String.format("%.5f", D));
                TelemetryUtil.packet.put("LL thetaLime", String.format("%.5f", thetaLime));
                TelemetryUtil.packet.put("LL tx", String.format("%.5f", result.getTx()));
                TelemetryUtil.packet.put("LL ty", String.format("%.5f", result.getTy()));

                TelemetryUtil.packet.put("LL Pose x", String.format("%.5f", estimatedLLPos.x));
                TelemetryUtil.packet.put("LL Pose y", String.format("%.5f", estimatedLLPos.y));

                TelemetryUtil.packet.put("LL globalLimelightEstimate x", String.format("%.5f", globalLLEstimate.x));
                TelemetryUtil.packet.put("LL globalLimelightEstimate y", String.format("%.5f", globalLLEstimate.y));
                TelemetryUtil.packet.put("LL globalLimelightEstimate heading", String.format("%.5f",globalLLEstimate.heading));

                Canvas fieldOverlay = TelemetryUtil.packet.fieldOverlay();
                DashboardUtil.drawRobot(fieldOverlay, new Pose2d(estimatedLLPos.x, estimatedLLPos.y, estimatedLLPos.heading), "#000000");

                TelemetryUtil.sendTelemetry();
            }
        }
    }
}
