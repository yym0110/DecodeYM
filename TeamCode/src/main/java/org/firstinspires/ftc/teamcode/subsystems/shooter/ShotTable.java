package org.firstinspires.ftc.teamcode.subsystems.shooter;
import java.util.TreeMap;

class ShotSetpoint {
    public final double flywheelVel;
    public final double hoodAngle;

    public ShotSetpoint(double shootVel, double hood) {
        this.flywheelVel = shootVel;
        this.hoodAngle = hood;
    }

    public ShotSetpoint lerp(ShotSetpoint other, double fraction) {
        return new ShotSetpoint(
                this.flywheelVel + (other.flywheelVel - this.flywheelVel) * fraction,
                this.hoodAngle + (other.hoodAngle - this.hoodAngle) * fraction
        );
    }
}

public class ShotTable {
    private final TreeMap<Double, ShotSetpoint> table = new TreeMap<>();
    public double flywheelMultiplier = 1.0;
    public double hoodMultiplier = 1.0;

    public ShotTable() {
        table.put(50.8, new ShotSetpoint(445, Math.toRadians(26.4)));
        table.put(60.8, new ShotSetpoint(465, Math.toRadians(39.0)));
        table.put(70.1, new ShotSetpoint(523, Math.toRadians(48.2)));
        table.put(80.6, new ShotSetpoint(525, Math.toRadians(48.5)));
        table.put(90.0, new ShotSetpoint(538, Math.toRadians(49.0)));
        table.put(98.4, new ShotSetpoint(546, Math.toRadians(49.3)));
        table.put(110.7, new ShotSetpoint(580, Math.toRadians(49.3)));
        table.put(122.7, new ShotSetpoint(607, Math.toRadians(49.6)));
        table.put(131.9, new ShotSetpoint(624, Math.toRadians(60.58)));
        table.put(135.4, new ShotSetpoint(638, Math.toRadians(60.58)));
    }

    public void addSetpoint(double distanceInches, ShotSetpoint setpoint) {
        table.put(distanceInches, setpoint);
    }

    public ShotSetpoint getSetpoint(double distanceInches) {
        Double lowIndex = table.floorKey(distanceInches); //this is to get the point above what we want for extrapolation
        Double higIndex = table.ceilingKey(distanceInches); //this is to get the point below what we want for extrapolation
        ShotSetpoint baseSetpoint;

        if (lowIndex == null && higIndex == null) return null; //case where both are null
        if (lowIndex == null) baseSetpoint = table.get(higIndex); //case where we only have highIndex
        else if (higIndex == null) baseSetpoint = table.get(lowIndex); //case where we only have lowIndex
        else if (lowIndex.equals(higIndex)) baseSetpoint = table.get(lowIndex); //case where both are the same
        else {
            double fraction = (distanceInches - lowIndex) / (higIndex - lowIndex); //interpolation based on the high and lower from input on the distance value
            baseSetpoint = table.get(lowIndex).lerp(table.get(higIndex), fraction); //smooths using linear interpolation between the previous factor to find the flywheelVelocity, hoodAngle, and time of flight.
        }

        return new ShotSetpoint(
                baseSetpoint.flywheelVel * flywheelMultiplier, //offset created just in case we are consistently off, we can tune this
                baseSetpoint.hoodAngle * hoodMultiplier //offset created just in case we are consistently off, we can tune this
        );
    }
}