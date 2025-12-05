package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.hardware.Gamepad;


import org.firstinspires.ftc.robotcore.external.Telemetry;


import java.lang.reflect.Array;
import java.util.ArrayList;

public class SelectionGUI {

    ArrayList<String> autoSteps;
    ArrayList<String> previousAutoSteps;

    int cursorPos = 0;

    public SelectionGUI(ArrayList<String> steps){
       autoSteps = steps;

       previousAutoSteps = steps;
    }

    public void update(Telemetry telemetry, Gamepad gamepad) {
        for(int i = 0; i < autoSteps.size(); i++) {
            telemetry.addData(String.valueOf(i + ". "), autoSteps.get(i));
        }

        telemetry.update();

        //move the cursor up in the list of steps
        if(gamepad.yWasPressed()) {
            if(cursorPos > 0) {
                cursorPos --;
            }
        }

        //move the cursor down in the list of steps
        if(gamepad.aWasPressed()) {
            if(cursorPos < autoSteps.size()) {
                cursorPos++;
            }
        }

        //if gamepad x was pressed remove the current auto step
        if(gamepad.xWasPressed()) {
            previousAutoSteps = autoSteps;
            autoSteps.remove(cursorPos);
            if(cursorPos > 0) {
                cursorPos--;
            } else{
                cursorPos++;
            }
        }

        //if gamepad b was pressed go back to the previous config
        if(gamepad.bWasPressed()) {
            ArrayList<String> temp = autoSteps;
            autoSteps = previousAutoSteps;
            previousAutoSteps = temp;

        }

    }

    public ArrayList<String> getAutoSteps() {
        return autoSteps;
    }
}
