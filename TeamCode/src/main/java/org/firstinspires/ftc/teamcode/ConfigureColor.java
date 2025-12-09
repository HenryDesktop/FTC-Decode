package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class ConfigureColor {

    RevColorSensorV3 colorSensor;

    public void init(HardwareMap hwMap){
        colorSensor = hwMap.get(RevColorSensorV3.class, "ColorSensor");
    }
    public double getDistance(){
        return colorSensor.getDistance(DistanceUnit.CM);
    }
}
