package org.firstinspires.ftc.teamcode.subsystems.shooter;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.utils.LogUtil;
import org.firstinspires.ftc.teamcode.utils.PID;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.Vector2;
import org.firstinspires.ftc.teamcode.utils.Vector3;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;
import org.firstinspires.ftc.teamcode.utils.priority.nPriorityServo;

import java.util.Vector;

@Config
public class Shooter {
    public enum State {
        IDLE,
        ACCEL,
        SHOOT,
        INDEX
    } State state = State.IDLE;

    private final Robot robot;
    private final Sensors sensors;
    private final DcMotorEx ms1, ms2;
    public final PriorityMotor flywheel;
    public final nPriorityServo flywheelBlocker, turret, hood, net;

    private boolean indexPrepareRequest = false, indexRequest = false;
    private boolean shootPrepareRequest = false, shootRequest = false;

    // velocity is in inches / second
    public static PID velocityPID = new PID (0.0, 0.001, 0.001);
    public static double velocityFFm = 0.0086733;
    public static double velocityFFb = 0.0414964;
    public static double velocityFilterLow = 0.05;
    public static double velocityFilterHigh = 0.5;
    public static double velocityFilterThresh = 10;
    public static double velocityHighPowerThresh = 4;
    public static double velocityNoSkipThresh = 45;
    public static double velocityNoSkipAccel = 0.7;
    private double targetVelocity = 0.0;
    private double filteredVelocity = 0.0;
    private double prevPow = 0;

    // autoaim stuff
    public Vector3 ballTarget = new Vector3(-60, 60,38.75 + 5);
    public PID turretPID = new PID (0.5, 0.0, 0.1);

    /*
    Hood / Velo
    Far: 1.34 / 100
    Middle: 1.0 / 70
    Close: 0.7 / 60
     */


    public Shooter(Robot robot) {
        this.robot = robot;

        this.sensors = robot.sensors;

        this.ms1 = robot.hardwareMap.get(DcMotorEx.class, "shooter1");
        this.ms2 = robot.hardwareMap.get(DcMotorEx.class, "shooter2");
        flywheel = new PriorityMotor(new DcMotorEx[]{ms1, ms2},"flywheel",3, 5, new double[] {1, -1}, robot.sensors);

        hood = new nPriorityServo(
            new Servo[]{robot.hardwareMap.get(Servo.class, "hood1"), robot.hardwareMap.get(Servo.class,"hood2")},
            "hood", nPriorityServo.ServoType.AXON_MINI,
            0.03, 0.38, 0.03,
            new boolean[] {false, true},
            2, 5
        );

        turret = new nPriorityServo(
            new Servo[]{robot.hardwareMap.get(Servo.class, "turret1"), robot.hardwareMap.get(Servo.class,"turret2")},
            "turret", nPriorityServo.ServoType.AXON_MINI,
            0.49, 0.51, 0.5,
            new boolean[] {false, false},
            2, 5
        );

        flywheelBlocker = new nPriorityServo(
                new Servo[]{robot.hardwareMap.get(Servo.class, "flywheelBlocker")},
                "flywheelBlocker", nPriorityServo.ServoType.AXON_MICRO,
                0, 0.7, 0.1,
                new boolean[] {false},
                2, 5
        );

        net = new nPriorityServo(
                new Servo[] {robot.hardwareMap.get(Servo.class, "net")},
                "net", nPriorityServo.ServoType.AXON_MINI,
                0, 1.0, 0.5,
                new boolean [] {false},
                2, 5
        );

        robot.hardwareQueue.addDevices(flywheel, hood, turret, flywheelBlocker, net);

        flywheel.motor[0].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.motor[0].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public double error;

    public void update() {
        switch (state){
            case IDLE:
                if(shootPrepareRequest){
                    // Calculation for targetVelocity goes here
                    state = State.ACCEL;
                }

                if(indexPrepareRequest){
                    // Calculation for targetVelocity goes here
                    state = State.ACCEL;
                }
                break;
            case ACCEL:
                if(atVel() && shootRequest){
                    state = State.SHOOT;
                }

                if(atVel() && indexRequest){
                    state = State.INDEX;
                }
                break;
            case SHOOT:
            case INDEX:
                setShooterBlocker(false);

                // TODO: Probably need to reorganize this to better differentiate the two actions, especially because indexing and shooting will lauch diff number of balls
                break;
        }


        // Flywheel Velocity PID
        if (targetVelocity <= 1) velocityPID.resetIntegral();
        else velocityPID.clipIntegral(-1, 1);
        double actualVelocity = robot.sensors.getFlywheelVelocity();
        if (Math.abs(actualVelocity - filteredVelocity) < velocityFilterThresh) {
            filteredVelocity = filteredVelocity * (1 - velocityFilterLow) + actualVelocity * velocityFilterLow;
        } else {
            filteredVelocity = filteredVelocity * (1 - velocityFilterHigh) + actualVelocity * velocityFilterHigh;
        }
        error = targetVelocity - filteredVelocity;
        double pow = velocityPID.update(error, 0.0, 1.0) + targetVelocity * velocityFFm + velocityFFb;
        if (error > velocityHighPowerThresh) pow = 1;
        if (filteredVelocity < velocityNoSkipThresh) {
            pow = Math.min(pow, prevPow + velocityNoSkipAccel * robot.sensors.loopTime);
        }
        flywheel.setTargetPower(pow);
        prevPow = pow;

        // Auto-aim (Disabled)
        /*
        goalDetector.update();
        if(goalDetector.isTagDetected() && Math.abs(goalDetector.getTx()) > limelightThresh && System.currentTimeMillis() - lastUpdateTime >= limelightTimeDelay){
            turretError = turret.getCurrentAngle() - Math.signum(goalDetector.getTx()) * limelightScalar;
            lastUpdateTime = System.currentTimeMillis();
        } else if (!goalDetector.isTagDetected()){
            turretError = 0;
        }
        turret.setTargetAngle(turretError);
         */

        // Turret Auto-Aim
        aimTurret(); // lmao


        TelemetryUtil.packet.put("Shooter : Flywheel Filtered Velocity", filteredVelocity);
        TelemetryUtil.packet.put("Shooter : Flywheel Target Velocity", targetVelocity);
        TelemetryUtil.packet.put("Shooter : Flywheel PID Power", pow * 100);
        TelemetryUtil.packet.put("Shooter : Turret Target Angle", turret.getTargetAngle());
    }

    public void indexPrepare() { indexPrepareRequest = true;}

    public void index() { indexRequest = true;}

    public void shootPrepare() { shootPrepareRequest = true;}

    public void shoot() {shootRequest = true;}

    public void setTurretAngle(double target_angle) {
        turret.setTargetAngle(target_angle);

        TelemetryUtil.packet.put("Shooter : turretAngle", target_angle);
        LogUtil.turretAngle.set(target_angle);
    }

    public void aimTurret() {
        Vector3 distance = new Vector3(ballTarget.getX() - sensors.getOdometryPosition().x, ballTarget.getY() - sensors.getOdometryPosition().y, 0);
        Vector3 ballExit2DSpd = new Vector3(getBallExitSpd() * distance.x/distance.getMag(), getBallExitSpd() * distance.y/distance.getMag(), 0);
        Vector3 tVel  = new Vector3(sensors.getVelocity().x, sensors.getVelocity().y, 0);
        // find a way to get robot angular velocity from drivetrain "turn"
        // Vector3 rVel = Vector3.cross(new Vector3(0, 0, ___), new Vector3(dLauncher * Math.cos(heading), dLauncher * Math.sin(heading), 0));
        Vector3 vel = Vector3.add(ballExit2DSpd, tVel); // .add(rVel);
        double rotate = Math.acos(Vector3.dot(vel, distance) / (vel.getMag() * distance.getMag())) * (Vector3.cross(vel, distance).z > 0 ? 1 : -1);
        setTurretAngle(Math.acos(distance.x/ distance.getMag()) + rotate); // works for red, if blue: 360 - Math.acos(...) + rotate

    }

    /**
     * The contents are just a placeholder for now
     * @return
     */
    public double getBallExitSpd() {
        return 639.899748567 / 14 * sensors.getVoltage();
        // the ~640 is just from the random 640 in/sec that PJ said, I have no clue what the model will be like but I suspect linear or quadratic
    }

    public void setHoodAngle(double target_angle) {
        hood.setTargetAngle(target_angle);

        TelemetryUtil.packet.put("Shooter : hoodAngle", target_angle);
        LogUtil.hoodAngle.set(target_angle);
    }

    public void setTargetVelocity(double targetVelocity) { this.targetVelocity = targetVelocity; }

    public double getTargetVelocity() { return targetVelocity; }

    public double getFilteredVelocity() { return filteredVelocity; }

    public void setShooterBlocker (boolean on) {flywheelBlocker.setTargetAngle (on ? 2.1 : -0.2);}

    public boolean atVel () {return Math.abs(error) < 1.0;}
}
