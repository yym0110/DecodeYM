package org.firstinspires.ftc.teamcode.subsystems.shooter;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.LogUtil;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;
import org.firstinspires.ftc.teamcode.utils.priority.nPriorityServo;

public class Shooter {
    private final Robot robot;

    private final DcMotorEx ms1, ms2;

    private final nPriorityServo turret, hood, cloth;
    private final PriorityMotor flywheel;

    public Shooter(Robot robot) {
        this.robot = robot;

        this.ms1 = robot.hardwareMap.get(DcMotorEx.class, "shooter1");
        this.ms2 = robot.hardwareMap.get(DcMotorEx.class, "shooter2");
        flywheel = new PriorityMotor(new DcMotorEx[]{ms1, ms2},"flywheel",3, 5, new double[] {1, -1}, robot.sensors);
        cloth = new nPriorityServo(
            new Servo[]{robot.hardwareMap.get(Servo.class, "cloth")},
            "cloth", nPriorityServo.ServoType.AXON_MINI,
            0, 1, 0.5,
            new boolean[] {false},
            2, 5
        );
        hood = new nPriorityServo(
            new Servo[]{robot.hardwareMap.get(Servo.class, "hood")},
            "hood", nPriorityServo.ServoType.AXON_MINI,
            0, 1, 0.5,
            new boolean[] {false},
            2, 5
        );
        turret = new nPriorityServo(
            new Servo[]{robot.hardwareMap.get(Servo.class, "turret1"), robot.hardwareMap.get(Servo.class,"turret2")},
            "turret", nPriorityServo.ServoType.AXON_MINI,
            0, 1, 0.5,
            new boolean[] {false, false},
            2, 5
        );

        robot.hardwareQueue.addDevices(flywheel, cloth, hood, turret);
    }

    public void update() {
        // TODO Flywheel velocity PID
    }

    public void setTurretPos(double target_angle) {
        turret.setTargetAngle(target_angle);
        TelemetryUtil.packet.put("Shooter : turretAngle", target_angle);
        LogUtil.turretAngle.set(target_angle);
    }
    public void setHoodPos(double target_angle) {
        hood.setTargetAngle(target_angle);
        TelemetryUtil.packet.put("Shooter : hoodAngle", target_angle);
        LogUtil.hoodAngle.set(target_angle);
    }

    public void setClothPos(double target_angle){cloth.setTargetAngle(target_angle);}
    public void setShooterPower(double power){flywheel.setTargetPower(power);}

    public void aimAt(double target_x, double target_y, double target_z) { // TODO Calculations
        double curr_x = this.robot.sensors.getOdometryPosition().x;
        double curr_y = this.robot.sensors.getOdometryPosition().y;
        double max_velocity = 10;
        double shoot_distance = Math.sqrt((target_x - curr_x) * (target_x - curr_x) + (target_y - curr_y) * (target_y - curr_y));
        double shoot_theta = Math.atan(max_velocity - 9.8 * shoot_distance);
        // pull the turret heading from the limelight
    }
}
