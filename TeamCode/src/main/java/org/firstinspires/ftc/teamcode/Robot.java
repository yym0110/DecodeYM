package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.utils.Globals.START_LOOP;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.park.Park;
import org.firstinspires.ftc.teamcode.utils.priority.HardwareQueue;
import org.firstinspires.ftc.teamcode.vision.Vision;

public class Robot {
    public HardwareMap hardwareMap;
    public HardwareQueue hardwareQueue;

    public Sensors sensors;
    public Drivetrain drivetrain;
    public Intake intake;
    public Park park;

    public Robot(HardwareMap hardwareMap, Vision vision){
        this.hardwareMap = hardwareMap;
        hardwareQueue = new HardwareQueue();

        sensors = new Sensors(this);
        drivetrain = new Drivetrain(this, new Vision(hardwareMap));
        intake = new Intake(this);
        park = new Park(this);

    }

    public void update(){
        START_LOOP();

        sensors.update();
        drivetrain.update();
        intake.update();
        park.update();

        hardwareQueue.update();

    }
}
