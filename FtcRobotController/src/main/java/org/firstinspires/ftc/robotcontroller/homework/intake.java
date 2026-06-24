package org.firstinspires.ftc.robotcontroller.homework;

import com.qualcomm.robotcore.robot.Robot;

public class intake {

    public enum State{
        IDLE,
        INTAKE,
        SHOOT_FEED,

    }
    private boolean requestIntake = false, requestShoot = false, requestOff = false, reversed = false;

    public State state = State.IDLE;
    public final PriorityMotor roller, feed;

    public void update(){
        switch (state){
            case IDLE:
                roller.setTargetPower(0);

                if(requestIntake){
                    requestIntake = false;
                    state = State.INTAKE;
                }
                if (requestShoot) {
                    requestShoot = false;
                    state = State.SHOOT_FEED;
                }
                break;
            case INTAKE:
                requestIntake = false;

                if (requestOff) {
                    requestOff = false;
                    requestIntake = false;
                    requestShoot = false;
                    state = State.IDLE;
                }

                if (requestShoot) {
                    requestShoot = false;
                    state = State.SHOOT_FEED;
                }
                break;
            case SHOOT_FEED:
                requestShoot = false;
                feed.setTargetPower();

                if (requestOff) {
                    requestOff = false;
                    requestIntake = false;
                    requestShoot = false;
                    state = State.IDLE;
                }

                if (requestIntake) {
                    requestIntake = false;
                    state = State.INTAKE;
                }
                break;
        }
    }
}
