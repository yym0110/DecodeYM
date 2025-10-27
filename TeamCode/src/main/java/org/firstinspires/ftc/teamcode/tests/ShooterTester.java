package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;

public class ShooterTester extends LinearOpMode {
    private Robot robot;
    private Shooter shooter;

    public static double turretAngle = 0.0, hoodAngle = 0.0, flywheelVelocity = 0.0, flywheelPower = 0.0;
    public static boolean useVelocity = false;

    public void runOpMode(){
        robot = new Robot(hardwareMap);
        shooter = new Shooter(robot);

        while(opModeInInit()){
            robot.update();
        }

        // TODO: Update ShooterTester to
        while(opModeIsActive()){
            shooter.setTurretAngle(turretAngle);
            shooter.setHoodAngle(hoodAngle);

            if(useVelocity) {
                shooter.setTargetVelocity(flywheelVelocity);
                useVelocity = false;
            }

            robot.update();
        }
    }
}
