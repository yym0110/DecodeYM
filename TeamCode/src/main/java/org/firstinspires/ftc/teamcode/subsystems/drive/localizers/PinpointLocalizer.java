package org.firstinspires.ftc.teamcode.subsystems.drive.localizers;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.utils.Pose2d;

public class PinpointLocalizer extends Localizer{
    private GoBildaPinpointDriver pinpoint;
    public PinpointLocalizer (HardwareMap hardwareMap, Sensors sensors, Drivetrain drivetrain, String color, String expectedColor){
        super(sensors, drivetrain, color, expectedColor);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(72, -160);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
    }

    public void setPose(double x, double y, double h){
        super.setPose(x, y, h);
        pinpoint.setPosition(new Pose2d(x, y, h));
    }

    public void setPoseEstimate (Pose2d pose2d){
        super.setPoseEstimate(pose2d);
        pinpoint.setPosition(pose2d);
    }

    private double lastUpdateTime = System.currentTimeMillis();

    public void update(){
        lastUpdateTime = System.currentTimeMillis();

        pinpoint.update();

        currentPose.x = x = pinpoint.getPosX();
        currentPose.y = y = pinpoint.getPosY();
        currentPose.heading = heading = pinpoint.getHeading();

        relCurrentVel.x = pinpoint.getVelX();
        relCurrentVel.y = pinpoint.getVelY();
        relCurrentVel.heading = Math.atan2(relCurrentVel.x, relCurrentVel.y);
    }

    public double getStaleness() { return System.currentTimeMillis() - lastUpdateTime;}
}
