package com.titanrobotics2022.geometry.experimentalgeometry;

public class Cylinderical implements CoordinateSystem, Vector3DOperation<Cylinderical> {
    /** Origin (coordinates: 0, 0, 0). */
    public static final Cylinderical ZERO = new Cylinderical(0, 0, 0);

    /** A vector with all coordinates set to NaN. */
    public static final Cylinderical NAN = new Cylinderical(Double.NaN, Double.NaN, Double.NaN);

    public final double r, theta, z;

    public Cylinderical(double r, double theta, double z)
    {
        this.r = r;
        this.theta = theta;
        this.z = z;
    }

    @Override
    public Cylinderical plus(Cylinderical rhs) {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public Cylinderical minus(Cylinderical rhs) {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public Cylinderical scalarMultiply(double scalar) {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public Cylinderical scalarDivide(double divisor) {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public Cylinderical negate() {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public double magnitude() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public double magnitudeSquared() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public Cylinderical unitVector() {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public double dot(Cylinderical rhs) {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public Cylinderical projectOnto(Cylinderical otherVec) {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public Cylinderical cross(Cylinderical rhs) {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public int getDimension() {
        // TODO Auto-generated method stub
        return 0;
    }
}
