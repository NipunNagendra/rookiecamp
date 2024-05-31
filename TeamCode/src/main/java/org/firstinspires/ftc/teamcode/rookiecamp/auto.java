package org.firstinspires.ftc.teamcode.rookiecamp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.rookiecamp.subsystems.Drive;
import org.firstinspires.ftc.teamcode.rookiecamp.subsystems.Manipulators;
import org.firstinspires.ftc.teamcode.rookiecamp.util.PID;
import org.firstinspires.ftc.teamcode.rookiecamp.util.Pose;

@Autonomous(name = "auto 1", group = "RookieCamp")
@Config
public class auto extends LinearOpMode {
    public static double point1x;

    public static double point2x;

    public static double point3x;

    public static double point1y;

    public static double point2y;

    public static double point3y;

    public static double point1h;

    public static double point2h;

    public static double point3h;

    Drive robotDrive;
    Manipulators robotManipulators;
    Pose position = new Pose(0, 0, 0);
    double leftPower = 0, rightPower = 0;
    double timeout = 5000;
    boolean loopIsActive = true;
    double threshold = 2, angleThreshold = 2;
    public static double kPd = 0.1, kId = 0.1, kDd = 0.1;
    public static double kPa = 0.1, kIa = 0, kDa = 0.1;
    enum state {
        IDLE,
        INTAKE,
        SHOOT,
        DEPO,
        RETURN
    };
    state currentState = state.IDLE;
    @Override
    public void runOpMode() {
        robotDrive = new Drive(hardwareMap, 0, 0, 0);
        robotManipulators = new Manipulators(hardwareMap);
        waitForStart();
        PID distanceController = new PID(kPd, kId, kDd);
        PID angleController = new PID(kPa, 0, kDa);
        ElapsedTime timer = new ElapsedTime();
//----------------------------------------------------------------------


        switch (currentState) {
            case SHOOT:
                currentState = state.DEPO;

            case DEPO:
                // skystone position 0 here

                currentState = state.INTAKE;

            case INTAKE:
                // foundation move code
                currentState = state.RETURN;
                break;
            case RETURN:
                // park the bot
                break;
        }


        double targetX = 50;
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
        }

        robotManipulators.setFlywheelPower(1);
        sleep(1000);
        robotManipulators.setFlywheelPower(0);
        //PID to Point
        targetX = 50;
        targetY = 50;
        targetAngle = 0;
        distanceController = new PID(kPd, kId, kDd);
        angleController = new PID(kPa, 0, kDa);
        timer = new ElapsedTime();
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
        }

    }

}