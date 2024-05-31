package org.firstinspires.ftc.teamcode.rookiecamp.util;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class PID {
    double kP, kI, kD;
    ElapsedTime time = new ElapsedTime();
    int accuracy = 3;
    double previousTime = 0, previousError = 0;
    double p = 0, i = 0, d = 0;
    double max_i = 0.2, min_i = -0.2;
    double power;
    double error;

    public PID(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.error = 0;

        time.reset();
    }

    public void update(double target, double state) {
        double currentTime = time.milliseconds();
        error = target - state;

        p = kP * error;

        i += kI * (error * (currentTime - previousTime));

        Range.clip(i, min_i, max_i);

        d = kD * (error - previousError) / (currentTime - previousTime);


        previousError = error;
        previousTime = currentTime;
    }

    public double calculate(double target, double state) {
        update(target, state);
        power = p + i + d;
        return power;
    }

    public double getError() {
        return error;
    }

}
