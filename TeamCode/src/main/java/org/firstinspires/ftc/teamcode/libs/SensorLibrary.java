package org.firstinspires.ftc.teamcode.libs;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class SensorLibrary {
    MB1242 Mb;
    public MB1242 ultrasonicSensor;

    public DistanceSensor dsSensor;

    public static double dsInvisibleWallThreshold = 0;
    public static double ultrasonicInvisibleWallThreshold = 0;


    public SensorLibrary(HardwareMap hardwareMap) {
        ultrasonicSensor = hardwareMap.get(MB1242.class, "ultrasonicSensor");
    }

    public double getUltrasonicDistance() {
        return Mb.getDistance(DistanceUnit.CM);
    }

    public boolean invisibleWallDetect()
    {
        if (dsSensor.getDistance(DistanceUnit.CM) < dsInvisibleWallThreshold && ultrasonicSensor.getDistance(DistanceUnit.CM) < ultrasonicInvisibleWallThreshold){
            return true;
        }
        return false;
    }
}

