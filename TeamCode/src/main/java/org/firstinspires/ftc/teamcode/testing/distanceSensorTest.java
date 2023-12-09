package org.firstinspires.ftc.teamcode.testing;
import com.ThermalEquilibrium.homeostasis.Filters.FilterAlgorithms.KalmanFilter;
import com.ThermalEquilibrium.homeostasis.Filters.FilterAlgorithms.LowPassFilter;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.acmerobotics.dashboard.FtcDashboard;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="distanceSensorTest", group="TeleOp")
public class distanceSensorTest extends LinearOpMode {

    double[] motorPower = {0, 0, 0, 0};
    public static double a = 0.9; // low pass gain, must be within 0 < x < 1
    public static double Q = 0.3; // High values put more emphasis on the sensor.
    public static double R = 3; // High Values put more emphasis on regression.
    public static int N = 3; // The number of estimates in the past we perform regression on.
    KalmanFilter kalmanFilter = new KalmanFilter(Q,R,N);
    LowPassFilter lowPassFilter = new LowPassFilter(a);

    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        DistanceSensor ds = hardwareMap.get(DistanceSensor.class, "ds");

        while (!isStopRequested()) {

            double currentValue = ds.getDistance(DistanceUnit.CM);
            double estimate_low = lowPassFilter.estimate(currentValue);
            double estimate_kalman = kalmanFilter.estimate(currentValue);
            telemetry.addData("Distance(cm) noisy", currentValue);
            telemetry.addData("Distance(cm) smoothed low-pass", estimate_low);
            telemetry.addData("Distance(cm) smoothed kalman", estimate_kalman);
            telemetry.update();
        }

    }
}