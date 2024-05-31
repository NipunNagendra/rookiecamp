package org.firstinspires.ftc.teamcode.rookiecamp.util;

public class Pose2d {
    public double x;
    public double y;
    public double heading;

    public Pose2d(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    public double getX(){
        return this.x;
    }
    public double getY(){
        return this.y;
    }
    public double getHeading(){
        return this.heading;
    }
    public Pose2d add(Pose2d other) {
        return new Pose2d(this.x + other.x, this.y + other.y, this.heading + other.heading);
    }

    public Pose2d sub(Pose2d other) {
        return new Pose2d(this.x - other.x, this.y - other.y, this.heading - other.heading);
    }

    public Pose2d mul(double scalar) {
        return new Pose2d(this.x * scalar, this.y * scalar, this.heading * scalar);
    }

    public Pose2d div(double scalar) {
        return new Pose2d(this.x / scalar, this.y / scalar, this.heading / scalar);
    }

    public double dot(Pose2d other) {
        return this.x * other.x + this.y * other.y + this.heading * other.heading;
    }

    public double cross(Pose2d other) {
        return this.x * other.y - this.y * other.x;
    }

    public double mag() {
        return Math.sqrt(this.x * this.x + this.y * this.y + this.heading * this.heading);
    }

    public Pose2d normalize() {
        return this.div(this.mag());
    }

    public double angle() {
        return Math.atan2(this.y, this.x);
    }

    public Pose2d rotate(double angle) {
        double cos = Math.cos(angle);
        double sin = Math.sin(angle);
        return new Pose2d(this.x * cos - this.y * sin, this.x * sin + this.y * cos, this.heading);
    }

    public Pose2d project(Pose2d other) {
        return other.mul(this.dot(other) / other.mag());
    }
}