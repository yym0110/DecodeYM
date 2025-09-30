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
    public final PriorityMotor rollerMotor;
    public final PriorityMotor transferMotor;
    public final PriorityCRServo feedServo;

    public enum State {
        IDLE,
        INTAKE,
        SORT_FEED,
        SORT_WAIT,
        SHOOT_FEED,
        SHOOT_WAIT,
    }

    public State state = State.IDLE;

    public Intake(Robot robot) {
        this.robot = robot;

        rollerMotor = new PriorityMotor(
            new DcMotorEx[] {robot.hardwareMap.get(DcMotorEx.class, "roller")},
            "roller", 2, 5,
            new double[] {1}, robot.sensors
        );
        transferMotor = new PriorityMotor(
            new DcMotorEx[] {robot.hardwareMap.get(DcMotorEx.class, "transfer")},
            "transfer", 2, 5,
            new double[] {1}, robot.sensors
        );
        feedServo = new PriorityCRServo(
            robot.hardwareMap.get(CRServo.class, "feed"),
            "feed",
            1, 5
        );

        robot.hardwareQueue.addDevices(rollerMotor, feedServo);
    }

    public void update() {
        switch (this.state) {
            case IDLE: {
                // TODO Not spinning roller
                break;
            }
            case INTAKE: {
                // TODO Spin roller
                break;
            }
            case SORT_FEED: {
                // TODO Hit 1 ball into shooter
                break;
            }
            case SORT_WAIT: {
                // TODO Buffer state
                break;
            }
            case SHOOT_FEED: {
                // TODO Shoot ball
                break;
            }
            case SHOOT_WAIT: {
                // TODO Buffer state
                break;
            }
        }

        this.updateTelemetry();
    }

    private void updateTelemetry() {
        TelemetryUtil.packet.put("Intake : state", this.state);
        LogUtil.intakeState.set(this.state.toString());
    }
}
