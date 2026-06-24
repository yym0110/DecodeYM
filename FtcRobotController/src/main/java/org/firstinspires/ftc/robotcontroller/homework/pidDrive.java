package org.firstinspires.ftc.robotcontroller.homework;

public class pidDrive {
    public double P;
    public double I;
    public double D;
    public void PID(double p, double i, double d){
        P = p;
        I = i;
        D = d;
    }

    private double integral = 0.0;
    private double proportion = 0.0;
    private double derivative = 0.0;
    private double lastError = 0.0;
    private double loopTime = 0.0;
    private long lastLoop = System.nanoTime();

    public void resetIntegral(){
        integral = 0;
    }

    public double getIntegral(){
        return integral;
    }
    public void clipIntegral(double min, double max){
        integral = Utils.minMaxClip(integral, min, max);
    }
    public double update(double error, double min, double max){
        long currentTime = System.nanoTime();
        loopTime = (currentTime - lastLoop) / 1.0e9;
        lastLoop = currentTime;

        proportion = P * error;
        integral += error * I * loopTime;
        derivative = D * (error - lastError) / loopTime;

        return Utils.minMaxClip(proportion + integral + derivative, min, max);
    }

    public void updatePid(double p, double i , double d){
        this.P = p;
        this.I = i;
        this.D = d;
    }
}
