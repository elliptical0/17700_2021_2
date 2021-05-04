package org.firstinspires.ftc.teamcode;

public class Transform {
    public Vector2 pos;
    public double head;

    public Transform(Vector2 pos, double head) {
        this.pos = pos;
        this.head = head;
    }

    public Transform(double x, double y, double head) {
        this.pos = new Vector2(x, y);
        this.head = head;
    }

    public void set(Vector2 pos, double head) {
        this.pos = pos;
        this.head = head;
    }

    public void set(double x, double y, double head) {
        this.pos = new Vector2(x, y);
        this.head = head;
    }

    public Transform add(Transform t) {
        return new Transform(pos.add(t.pos), head + t.head);
    }

    public Transform add(Vector2 pos, double head) {
        return new Transform(this.pos.add(pos), this.head + head);
    }

    public String toString() {
        return pos + ", " + head;
    }
}
