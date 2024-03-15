package org.firstinspires.ftc.teamcode.libs;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Arrays;
import java.util.List;


public class CustomMecanumKinematics {
    static Movement driveBase;

    public static double lateralMultiplier = 1.0;
    public static double trackWidth = 1.0;
    public static double wheelBase = 1.0;
    public static double VX_WEIGHT = 1.0;
    public static double VY_WEIGHT = 1.0;
    public static double OMEGA_WEIGHT = 1.0;


    public CustomMecanumKinematics(HardwareMap hardwareMap) {
        driveBase = new Movement(hardwareMap);
    }

    public void setDrivePower(Pose2d drivePower, double powerProportion) {

    }

    public double[] setWeightedDrivePower(Pose2d drivePower) {
        Pose2d vel = drivePower;

        if (Math.abs(drivePower.getX()) + Math.abs(drivePower.getY()) + Math.abs(drivePower.getHeading()) > 1) {
            double denom = VX_WEIGHT * Math.abs(drivePower.getX()) + VY_WEIGHT * Math.abs(drivePower.getY()) + OMEGA_WEIGHT * Math.abs(drivePower.getHeading());
            vel = new Pose2d(
                    VX_WEIGHT * drivePower.getX(),
                    VY_WEIGHT * drivePower.getY(),
                    OMEGA_WEIGHT * drivePower.getHeading()
            ).div(denom);
        }

        return inverseKinematicModel(drivePower, trackWidth, wheelBase, lateralMultiplier);
    }


    public static double[] inverseKinematicModel(Pose2d robotVel, double trackWidth, double wheelBase, double lateralMultiplier) {
        double d = (trackWidth + wheelBase) / 2.0;
        return new double[]{
                robotVel.getX() - lateralMultiplier * robotVel.getY() - d * robotVel.getHeading(),
                robotVel.getX() + lateralMultiplier * robotVel.getY() - d * robotVel.getHeading(),
                robotVel.getX() - lateralMultiplier * robotVel.getY() + d * robotVel.getHeading(),
                robotVel.getX() + lateralMultiplier * robotVel.getY() + d * robotVel.getHeading()
        };
    }

    public static Pose2d forwardKinematicModel(List<Double> wheelVelocities, double trackWidth, double wheelBase, double lateralMultiplier) {
        double d = (trackWidth + wheelBase) / 2.0;
        return new Pose2d(
                (wheelVelocities.get(0) + wheelVelocities.get(1) + wheelVelocities.get(2) + wheelVelocities.get(3)) / 4.0,
                (wheelVelocities.get(0) - wheelVelocities.get(1) - wheelVelocities.get(2) + wheelVelocities.get(3)) / 4.0 * lateralMultiplier,
                (wheelVelocities.get(0) - wheelVelocities.get(1) + wheelVelocities.get(2) - wheelVelocities.get(3)) / (4.0 * d)
        );

    }

}



