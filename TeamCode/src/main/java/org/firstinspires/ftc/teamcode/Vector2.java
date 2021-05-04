package org.firstinspires.ftc.teamcode;

/**
 * Two-dimensional vector.
 * @author TM
 */
public class Vector2 {
    double x;
    double y;

    public Vector2(double x_, double y_) {
        x = x_;
        y = y_;
    }

    /**
     * Creates a unit vector in direction theta
     * @param theta in radians
     */
    public Vector2(double theta) {
        x = Math.cos(theta);
        y = Math.sin(theta);
    }

    public void set(Vector2 vector) {
        x = vector.x;
        y = vector.y;
    }
    public void set(double x_, double y_) {
        x = x_;
        y = y_;
    }

    public boolean equals(Vector2 vector) {
        return((vector.x == x) && (vector.y == y));
    }
    public boolean equals(double x_, double y_) {
        return((x_ == x) && (y_ == y));
    }

    public Vector2 add(Vector2 vector) {
        return new Vector2(x + vector.x, y + vector.y);
    }
    public Vector2 add(double x_, double y_) {
        return new Vector2(x + x_, y + y_);
    }

    public Vector2 subtract(Vector2 vector) {
        return new Vector2(x - vector.x, y - vector.y);
    }
    public Vector2 subtract(double x_, double y_) {
        return new Vector2(x - x_, y - y_);
    }

    public Vector2 multiply(Vector2 vector) {
        return new Vector2(x * vector.x, y * vector.y);
    }
    public Vector2 multiply(double x_, double y_) {
        return new Vector2(x * x_, y * y_);
    }
    public Vector2 multiply(double s) {
        return new Vector2(x * s, y * s);
    }

    public Vector2 divide(Vector2 vector) {
        return new Vector2(x / vector.x, y / vector.y);
    }
    public Vector2 divide(double x_, double y_) {
        return new Vector2(x / x_, y / y_);
    }

    public double length() {
        return (Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2)));
    }

    /**
     * Rotate counter-clockwise.
     * @param theta in radians.
     */
    public Vector2 rotate(double theta) {
        return new Vector2(Math.cos(theta) * x - Math.sin(theta) * y,
                Math.sin(theta) * x + Math.cos(theta) * y);
    }
    /**
     * Rotate clockwise.
     * @param theta in radians.
     */
    public Vector2 rotateClock(double theta) {
        return rotate(-theta);
    }

    /**
     * @return in radians.
     */
    public double direction() {
        return Math.atan2(y, x);
    }

    public String toString() {
        return x + ", " + y;
    }
}