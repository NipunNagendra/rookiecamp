package org.firstinspires.ftc.teamcode.drivepp;

import static org.firstinspires.ftc.teamcode.drivepp.purePursuitLibrary.injectWaypoints;

public class pointsTest {
    public static void main(String[] args) {
        Waypoint[] waypoints = {
                new Waypoint(0, 0),
                new Waypoint(1, 1),
                new Waypoint(2, 0)
        };

        double spacing = 0.5;
        Waypoint[] newWaypoints = injectWaypoints(waypoints);

        for (Waypoint wp : newWaypoints) {
            System.out.println("X: " + wp.x + ", Y: " + wp.y);
        }
    }

}
