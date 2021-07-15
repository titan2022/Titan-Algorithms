package com.titanrobotics2022.geometry.experimentalgeometry;

public class Spherical implements CoordinateSystem, Vector3DOperation<Spherical>{
    /** Origin (coordinates: 0, 0, 0). */
    public static final Spherical ZERO = new Spherical(0, 0, 0);

    /** A vector with all coordinates set to NaN. */
    public static final Spherical NAN = new Spherical(Double.NaN, Double.NaN, Double.NaN);

    public final double rou, theta, phi;

    public Spherical(double rou, double theta, double phi)
    {
        this.rou = rou;
        this.theta = theta;
        this.phi = phi;
    }

    @Override
    public Spherical plus(Spherical rhs) {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public Spherical minus(Spherical rhs) {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public Spherical scalarMultiply(double scalar) {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public Spherical scalarDivide(double divisor) {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public Spherical negate() {
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
    public Spherical unitVector() {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public double dot(Spherical rhs) {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public Spherical projectOnto(Spherical otherVec) {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public Spherical cross(Spherical rhs) {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public int getDimension() {
        // TODO Auto-generated method stub
        return 0;
    }
}
