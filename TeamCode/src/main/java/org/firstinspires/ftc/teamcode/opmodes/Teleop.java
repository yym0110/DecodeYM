package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.LogUtil;
import org.firstinspires.ftc.teamcode.utils.RunMode;

//@TeleOp(name = "A. Teleop")
public class Teleop extends LinearOpMode {
    public void runOpMode() {
        Globals.RUNMODE = RunMode.TELEOP;
        Robot robot = new Robot(hardwareMap);
        robot.setStopChecker(this::isStopRequested);

        while (opModeInInit()) {
            robot.update();
        }

        if (!isStopRequested()) LogUtil.init();
        LogUtil.drivePositionReset = true;

        while (!isStopRequested()) {
            robot.update();
        }
    }
}
