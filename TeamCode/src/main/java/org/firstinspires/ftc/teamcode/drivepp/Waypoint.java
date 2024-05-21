package org.firstinspires.ftc.teamcode.drivepp;

public class Waypoint {
    double x;
    double y;
    double distance;
    double curvature;
    double velocity;

    public Waypoint(double x, double y) {
        this.x = x;
        this.y = y;
    }
    public void setDistance(double distance){
        this.distance = distance;
    }
    public void setCurvature(double curvature){
        this.curvature = curvature;
    }
}
