package org.firstinspires.ftc.teamcode.subsystems.park;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.utils.LogUtil;
import org.firstinspires.ftc.teamcode.utils.PID;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.Utils;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityCRServo;


@Config
public class Park {
    private final Robot robot;

    private final PriorityCRServo park;

    //bellypan positions
    //horizontal is 0
    public static double target_pos = 0;//target position and position can be thought of as height of slides if the robot was flipped upside down
    public static double pos = 0;

    //positive power corresponds to pushing the robot up/pushing the slides down
    public static double threshold = 0.3;
    public static PID parkPID = new PID (0.01, 0.0, 0.0);

    public static double forcePullInPower = -0.2;
    public static double stayUpPower = 0.2;

    public static double kstatic = 0.2;

    public static double maxLength = 17; //?


    public enum State {
        IDLE,
        PULL_IN,
        EXTEND,
        WAIT_AT_TOP,
        MANUAL_CONTROL
    }

    private boolean nextState = false;
    private boolean previousState = false;

    public State state = State.IDLE;

    public Park(Robot robot) {
        this.robot = robot;

        park = new PriorityCRServo(
            new CRServo[]{robot.hardwareMap.get(CRServo.class, "park1"), robot.hardwareMap.get(CRServo.class,"park2")},
            "park", PriorityCRServo.ServoType.AXON_MAX,
            new boolean[]{false, true},
            2, 5
        );

        robot.hardwareQueue.addDevice(park);
    }

    public void update() {

        switch (state) {
            case IDLE: {
                setTargetPos(0);
            }
            case PULL_IN: {
                setTargetPos(-0.5);
            }
            case EXTEND: {
                setTargetPos(maxLength);
                if(inPosition(threshold)) { state = State.WAIT_AT_TOP; }
            }
            case WAIT_AT_TOP: {
                setTargetPos(maxLength);
            }
            case MANUAL_CONTROL: {
                break;
            }
        }

        //park position in inches that the bellypan is extended
        pos = getParkPos();
        //positive error leads to positive power and vice versa
        double pow = parkPID.update((target_pos - pos), -1, 1);

        if (inPosition(threshold)) parkPID.resetIntegral();

        //slides need to go down so power is positive
        pow += kstatic * Math.signum(target_pos - pos);

        if(target_pos <= 0 && pos < 0.5) { pow = forcePullInPower; }

        if(target_pos == maxLength && inPosition(threshold)) { pow = stayUpPower; }

        park.setTargetPower(Utils.minMaxClip(pow, -1,1));

        this.updateTelemetry();
    }

    private void updateTelemetry() {
        TelemetryUtil.packet.put("Park : state", this.state);
        TelemetryUtil.packet.put("Park : position", pos);
        LogUtil.parkState.set(this.state.toString());
        LogUtil.parkAngle.set(pos);
    }

    public void setTargetPos(double target) { target_pos = Utils.minMaxClip(target, -0.5, maxLength); }

    public double getParkPos() { return robot.sensors.getParkPos(); }

    public boolean inPosition(double t) {return Math.abs(target_pos - pos) < t;}
}
