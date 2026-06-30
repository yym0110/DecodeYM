package org.firstinspires.ftc.robotcontroller.homework;

public class Shooter_hw2 {

    public enum State {
        IDLE,
        READY,
        SHOOT,
    }
    public State state = State.IDLE;

    private final Robot robot;
    public final Flywheel flywheel;
    public boolean shootRequest = false;
    public double xDiff = 0.0; // how far robot is from target x value
    public double yDiff = 0.0; // how far robot is from target y value
    public double targetVelo = 0.0;

    public double targetAngle = 0.0;

    switch (state){
        case IDLE: {
            flywheel.setTargetVelocity(targetVelo);
            if(shootRequest){
                state = State.READY;
            }
        }
        case READY: {
            if(targetVelo > /*some threshold*/){
                state = State.SHOOT;
            }
        }
        case SHOOT: {
            shootRequest = false;
            targetVelo = Math.sqrt(xDiff * xDiff + yDiff * yDiff);
            flywheel.setTargetVelocity(targetVelo);
        }
    }
}
