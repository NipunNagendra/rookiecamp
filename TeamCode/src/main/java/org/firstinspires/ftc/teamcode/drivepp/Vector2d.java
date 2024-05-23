package org.firstinspires.ftc.teamcode.drivepp;

public class Vector2d {
    public double x;
    public double y;

    public Vector2d(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public double getX(){
        return this.x;
    }
    public double getY(){
        return this.y;
    }
    public Vector2d add(Vector2d other) {
        return new Vector2d(this.x + other.x, this.y + other.y);
    }

    public Vector2d sub(Vector2d other) {
        return new Vector2d(this.x - other.x, this.y - other.y);
    }

    public Vector2d mul(double scalar) {
        return new Vector2d(this.x * scalar, this.y * scalar);
    }

    public Vector2d div(double scalar) {
        return new Vector2d(this.x / scalar, this.y / scalar);
    }

    public double dot(Vector2d other) {
        return this.x * other.x + this.y * other.y;
    }

    public double cross(Vector2d other) {
        return this.x * other.y - this.y * other.x;
    }

    public double mag() {
        return Math.sqrt(this.x * this.x + this.y * this.y);
    }

    public Vector2d normalize() {
        return this.div(this.mag());
    }

    public double angle() {
        return Math.atan2(this.y, this.x);
    }

    public Vector2d rotate(double angle) {
        double cos = Math.cos(angle);
        double sin = Math.sin(angle);
        return new Vector2d(this.x * cos - this.y * sin, this.x * sin + this.y * cos);
    }

    public Vector2d project(Vector2d other) {
        return other.mul(this.dot(other) / other.dot(other));
    }

    public Vector2d reflect(Vector2d normal) {
        return this.sub(normal.mul(2 * this.dot(normal)));
    }

    public Vector2d lerp(Vector2d other, double t) {
        return this.mul(1 - t).add(other.mul(t));
    }

    public Vector2d slerp(Vector2d other, double t) {
        double theta = Math.acos(this.dot(other) / (this.mag() * other.mag()));
        return this.mul(Math.sin((1 - t) * theta) / Math.sin(theta)).add(other.mul(Math.sin(t * theta) / Math.sin(theta)));
    }

    public Vector2d clone() {
        return new Vector2d(this.x, this.y);
    }

    public String toString() {
        return "X: " + this.x + ", Y: " + this.y;
    }

}
