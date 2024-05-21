package org.firstinspires.ftc.teamcode.drivepp;

import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.apache.commons.math3.geometry.Vector;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;

import java.util.ArrayList;
import java.util.List;

public class purePursuitLibrary {
    public static double spacing = 3;
    public static Waypoint[] injectWaypoints(Waypoint[] points) {
        if (points.length < 2 || spacing <= 1) {
            return points;
        }

        List<Waypoint> newPoints = new ArrayList<>();
        double vectorMagnitude;
        int numPointsThatFit;
        for (int i = 0; i < (points.length - 1); i++) {
            Waypoint start_point = points[i];
            Waypoint end_point = points[i + 1];

            vectorMagnitude = calculateVectorMagnitude(start_point, end_point);
            numPointsThatFit = (int) Math.ceil(vectorMagnitude / spacing);
            double xIncrement = ((end_point.x - start_point.x) / vectorMagnitude) * spacing;
            double yIncrement = ((end_point.y - start_point.y) / vectorMagnitude) * spacing;

            for (int j = 0; j < numPointsThatFit; j++) {
                double newX = start_point.x + j * xIncrement;
                double newY = start_point.y + j * yIncrement;
                newPoints.add(new Waypoint(newX, newY));
            }

        }
        newPoints.add(points[points.length - 1]);
        return newPoints.toArray(new Waypoint[0]);

    }


    public static double calculateVectorMagnitude(Waypoint point1, Waypoint point2){
        return Math.sqrt(((point2.x-point1.x) * (point2.x-point1.x)) + ((point2.y-point1.y) * (point2.y-point1.y)));
    }


}