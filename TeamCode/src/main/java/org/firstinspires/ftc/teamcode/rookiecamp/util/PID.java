package org.firstinspires.ftc.teamcode.rookiecamp.util;

public class PID {
    double kP, kI, kD;


    public PID(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public double update(double target, double state) {

    }

}
