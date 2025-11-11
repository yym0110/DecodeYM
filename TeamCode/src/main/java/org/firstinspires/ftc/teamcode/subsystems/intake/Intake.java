package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.LogUtil;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityCRServo;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;

public class Intake {
    private final Robot robot;
    public final PriorityMotor roller, feed;

    private boolean requestIntake = false, requestShoot = false;

    public enum State {
        IDLE,
        INTAKE,
        SHOOT_FEED,
        TEST
    }

    public State state = State.IDLE;

    public Intake(Robot robot) {
        this.robot = robot;

        roller = new PriorityMotor(
            new DcMotorEx[] {robot.hardwareMap.get(DcMotorEx.class, "roller")},
            "roller", 2, 5,
            new double[] {-1}, robot.sensors
        );

        feed = new PriorityMotor (
                new DcMotorEx[] {robot.hardwareMap.get(DcMotorEx.class, "feed")},
                "feed", 2, 5,
                new double[] {1}, robot.sensors
        );

        robot.hardwareQueue.addDevices(roller, feed);
    }

    long launchTime = System.currentTimeMillis();

    public void update() {
        switch (state) {
            case IDLE: {
                // TODO Not spinning roller
                roller.setTargetPower(0.0);
                feed.setTargetPower(0.0);

                // TODO Disable Color Detection?
                robot.sensors.toggleColor(false);

                if (requestIntake) {
                    requestIntake = false;
                    state = State.INTAKE;
                }

                break;
            }
            case INTAKE: {
                roller.setTargetPower(0.7);
                feed.setTargetPower(0.3);

                // TODO Toggle Color Detection?

                if(requestShoot){
                    requestShoot = false;
                    state = State.SHOOT_FEED;
                    launchTime = System.currentTimeMillis();
                }

                break;
            }
            case SHOOT_FEED: {
                roller.setTargetPower(0.7);
                feed.setTargetPower(0.7);

                // Add launch time requirement because last ball theoretically will not trigger color sensor
                if(!robot.sensors.isBall() && System.currentTimeMillis() - launchTime >= 150) {
                    state = State.IDLE;
                }
            }
            case TEST: {
                break;
            }
        }

        this.updateTelemetry();
    }

    public void reqIntake(boolean req) {
        requestIntake = req;
    }

    public void reqShoot(boolean req){
        requestShoot = req;
    }

    private void updateTelemetry() {
        TelemetryUtil.packet.put("Intake : state", this.state);
        LogUtil.intakeState.set(this.state.toString());
    }
}
