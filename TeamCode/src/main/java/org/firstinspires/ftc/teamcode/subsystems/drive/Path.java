package org.firstinspires.ftc.teamcode.subsystems.drive;

import android.util.Log;

import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.Vector2;

import java.util.ArrayList;

class PathSegment {
    public final Spline spline;
    public final boolean reversed;
    public final double power;

    PathSegment (Spline s, boolean r, double p){
        spline = s;
        reversed = r;
        power = p;
    }
}

class PathData{
    Vector2 vel, accel;
    double r, power;
    boolean reversed;
    int index;

    public PathData (Vector2 vel, Vector2 accel, double r, double power, boolean reversed, int index) {
        this.vel = vel;
        this.accel = accel;
        this.r = r;
        this.power = power;
        this.reversed = reversed;
        this.index = index;
    }
}

public class Path {
    ArrayList <PathSegment> pathSegments;
    ArrayList <RepulsionPoint> repel;
    boolean reversed, completed;
    double power;
    int lastReachedIndex = 0;
    Pose2d lastPose;

    public Path (Pose2d p, ArrayList<RepulsionPoint> repel) {
        this.repel = repel;
        pathSegments = new ArrayList <>();
        reversed = false;
        completed = false;
        power = 1.0;
        lastPose = p.clone();
    }

    public Path addPoint(Pose2d p) {
        if(reversed) {
            p.heading += Math.PI;
        }

        Spline s = new Spline (lastPose, p);
        pathSegments.add(new PathSegment(s, reversed, 1.0));
        lastPose = p.clone();

        return this;
    }

    public Path addRepel(RepulsionPoint point){
        this.repel.add(point);
        return this;
    }

    public Path setReversed (boolean rev) {
        if (rev != reversed) {
            lastPose.heading += Math.PI;
        }
        reversed = rev;

        return this;
    }

    public static double k_p = 0.125;

    public Vector2 getVelocity (Spline s, double tau, Vector2 robot) {
        Vector2 v_t = s.getVel(tau);
        v_t.norm();
        // Log.i("Path v_t", v_t + "");

        Vector2 v_p = new Vector2(s.getPos(tau).x - robot.x, s.getPos(tau).y - robot.y);
        v_p.mul(k_p);
        // Log.i("Path v_p", v_p + "");

        Vector2 v_rep = new Vector2(0, 0);
        for(RepulsionPoint rep : repel) {
            Vector2 trep = new Vector2(robot.x - rep.x, robot.y - rep.y);
            double scale = Math.pow(Math.E, 1 - Math.pow(trep.mag()/rep.weight,2));
            trep.norm();
            trep.mul(scale);

            v_rep.add(trep);
        }
        // Log.i("Path v_rep", v_rep + "");

        return Vector2.add(v_t, Vector2.add(v_p, v_rep));
    }

    public Vector2 getAccel (Spline s, double tau, Vector2 robot, Vector2 vel) {
        Pose2d robotNext = new Pose2d(robot.x + vel.x * 0.001, robot.y + vel.y * 0.001);
        double tauNext = s.getT(robotNext);

        // Log.i("Path tau", tau + "");
        Log.i("Path robotCurr", robot + "");
        // Log.i("Path tauNext", tauNext + "");
        Log.i("Path robotNext", robotNext + "");

        Vector2 velNext = getVelocity (s, tauNext, new Vector2(robotNext.x, robotNext.y));

        return new Vector2 ((velNext.x - vel.x) / 0.001, (velNext.y - vel.y) / 0.001);
    }

    public PathData update (Pose2d robot) {
        PathSegment curr;
        int index = lastReachedIndex;

        while(index < pathSegments.size() && pathSegments.get(index).spline.getT(robot) == 1.0) {
            index++;
        }

        lastReachedIndex = index;

        for (int i = 0; i < pathSegments.size(); i++){
            Log.i("Path Spline " + i + " tau", pathSegments.get(i).spline.getT(robot) + "");
        }

        Log.i("Path chosen index", index + "");

        completed = index == pathSegments.size();
        if (completed) {
            return null;
        }

        curr = pathSegments.get(index);
        double tau = curr.spline.getT(robot);

        Vector2 vel = getVelocity (curr.spline, tau, new Vector2(robot.x, robot.y));
        Vector2 accel = getAccel (curr.spline, tau, new Vector2(robot.x, robot.y), vel);

        return new PathData(vel, accel, (vel.mag() * vel.mag() * vel.mag()) / (vel.x * accel.y - vel.y * accel.x), curr.power, curr.reversed, index);
    }
}
