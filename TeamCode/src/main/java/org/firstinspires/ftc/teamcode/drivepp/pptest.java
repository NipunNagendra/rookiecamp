package org.firstinspires.ftc.teamcode.drivepp;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drivepp.Waypoint;

@Config
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="pptest", group="Autonomous")
public class pptest extends LinearOpMode {
    Waypoint[] waypoints = new Waypoint[10];
    int waypointIndex = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        addWaypoint(10.0, 5.0);
        addWaypoint(15.0, 8.0);



        waitForStart();
        while (opModeIsActive()) {


        }
    }

    private void addWaypoint(double x, double y) {
        if (waypointIndex < waypoints.length) {
            waypoints[waypointIndex++] = new Waypoint(x, y);
        } else {
            telemetry.addData("Error", "Waypoint array is full!!!");
            telemetry.update();
        }
    }


}