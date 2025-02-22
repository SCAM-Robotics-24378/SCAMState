package org.firstinspires.ftc.teamcode.subsystem.util;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class Point {
    private double x;
    private double y;
    private double heading; // in radians

    // Constructor
    public Point(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    // Default Constructor
    public Point() {
        this(0.0, 0.0, 0.0);
    }

    public Pose2D toPose() {
        return new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.RADIANS, Math.toRadians(heading));
    }

    // Getters and Setters

    public double x() {
        return x;
    }

    public void setX(double x) {
        this.x = x;
    }

    public double y() {
        return y;
    }

    public void setY(double y) {
        this.y = y;
    }

    public double heading() {
        return heading;
    }

    public void setHeading(double heading) {
        this.heading = heading;
    }

    // Set the entire pose at once
    public void setPoint(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }
}