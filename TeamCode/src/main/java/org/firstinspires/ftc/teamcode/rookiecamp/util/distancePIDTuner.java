package org.firstinspires.ftc.teamcode.rookiecamp.util;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.rookiecamp.subsystems.Drive;
import org.firstinspires.ftc.teamcode.rookiecamp.util.PID;
import org.firstinspires.ftc.teamcode.rookiecamp.util.Pose;
@Autonomous(name = "distancePIDTuner", group = "RookieCamp")
@Config
public class distancePIDTuner extends LinearOpMode {
    Drive robotDrive;
    Pose position = new Pose(0, 0, 0);
    double leftPower = 0, rightPower = 0;
    double timeout = 15000;
    boolean loopIsActive = true;
    double threshold = 2, angleThreshold = 2;
    public static double kPd = 0.1, kId = 0.1, kDd = 0.1;
    public static double kPa = 0.1, kIa = 0, kDa = 0.1;

    @Override
    public void runOpMode() {
        robotDrive = new Drive(hardwareMap, 0, 0, 0);
        robotDrive.imu.resetYaw();
        waitForStart();
        PID distanceController = new PID(kPd, kId, kDd);
        PID angleController = new PID(kPa, 0, kDa);
        ElapsedTime timer = new ElapsedTime();
//----------------------------------------------------------------------
        double targetX = 0;
        double targetY = 50;
        double targetAngle = 0;

        while (loopIsActive && timer.milliseconds() <= timeout) {
            Pose poseEstimate = robotDrive.getPose();
            double robotX = poseEstimate.getX();
            double robotY = poseEstimate.getY();
            double robotTheta = poseEstimate.getHeading();

            double xError = targetX - robotX;
            double yError = targetY - robotY;
            double theta = Math.atan2(yError,xError);
            double distance = Math.hypot(xError, yError);
            double f, t;


            if (distance < threshold) {
                f = 0;
                t = angleController.calculate(targetAngle, robotTheta);
                double angleError = targetAngle - robotTheta;
                if(Math.abs(angleError) < angleThreshold){
                    loopIsActive = false;
                }
            } else {
                f = distanceController.calculate(0, distance);
                t = angleController.calculate(theta, robotTheta);
            }
            f *= Math.cos(Range.clip(angleController.getError(), -Math.PI/2, Math.PI/2));


            leftPower = Range.clip(f + t, -1.0, 1.0);
            rightPower = Range.clip(f - t, -1.0, 1.0);
            robotDrive.setRelativePower(leftPower, rightPower);

            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("h", poseEstimate.getHeading());
            telemetry.update();
            robotDrive.update();
        }
    }

}
