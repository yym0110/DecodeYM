package org.firstinspires.ftc.teamcode.subsystems.shooter;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.LogUtil;
import org.firstinspires.ftc.teamcode.utils.PID;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;
import org.firstinspires.ftc.teamcode.utils.priority.nPriorityServo;

public class Shooter {
    private final Robot robot;

    private final DcMotorEx ms1, ms2;

    /*
        Servo Zeroes
        turret: Relative to front of the robot, counterclockwise rotation is positive
        hood: Fully retracted
        cloth:
     */
    private final nPriorityServo turret, hood, cloth;
    public final PriorityMotor flywheel;

    public static PID velocityPID = new PID (1.0, 0.0, 0.0);
    private double targetVelocity = 0.0;

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
            new Servo[]{robot.hardwareMap.get(Servo.class, "hood1"), robot.hardwareMap.get(Servo.class,"hood2")},
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
        double error = targetVelocity - robot.sensors.getFlywheelVelocity();
        double pow = velocityPID.update(error, 0.0, 1.0);
        setShooterPower(pow);

        TelemetryUtil.packet.put("Flywheel Current Velocity", robot.sensors.getFlywheelVelocity());
        TelemetryUtil.packet.put("Flywheel Target Velocity", targetVelocity);
        TelemetryUtil.packet.put("Flywheel Velocity Error", error);
        TelemetryUtil.packet.put("Flywheel PID Power", pow);
    }

    /**
     * @param target_angle specifies rotation in XY plane [-180, 180] or [-PI, PI]
     */
    public void setTurretAngle(double target_angle) {
        turret.setTargetAngle(target_angle);

        TelemetryUtil.packet.put("Shooter : turretAngle", target_angle);
        LogUtil.turretAngle.set(target_angle);
    }

    /**
     * @param target_angle specifies rotation in Z plane [0, 90] or [0, PI/2]
     */
    public void setHoodAngle(double target_angle) {
        hood.setTargetAngle(target_angle);

        TelemetryUtil.packet.put("Shooter : hoodAngle", target_angle);
        LogUtil.hoodAngle.set(target_angle);
    }

    public void setClothPos(double target_angle){cloth.setTargetAngle(target_angle);}

    public void setShooterPower(double power) { flywheel.setTargetPower(power); }
    public void setTargetVelocity(double targetVelocity) { this.targetVelocity = targetVelocity; }
    public double getTargetVelocity() { return targetVelocity; }

    public void aimAt(double target_x, double target_y, double target_z) { // TODO Calculations
        double curr_x = this.robot.sensors.getOdometryPosition().x;
        double curr_y = this.robot.sensors.getOdometryPosition().y;
        double max_velocity = 10;
        double shoot_distance = Math.sqrt((target_x - curr_x) * (target_x - curr_x) + (target_y - curr_y) * (target_y - curr_y));
        double shoot_theta = Math.atan(max_velocity - 9.8 * shoot_distance);
        // pull the turret heading from the limelight
    }
}
