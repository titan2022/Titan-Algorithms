package com.titanrobotics2022.geometry.experimentalgeometry;

public class Cartesian3D implements CoordinateSystem, Vector3DOperation<Cartesian3D> {
    /** Origin (coordinates: 0, 0, 0). */
    public static final Cartesian3D ZERO = new Cartesian3D(0, 0, 0);

    /** A vector with all coordinates set to NaN. */
    public static final Cartesian3D NAN = new Cartesian3D(Double.NaN, Double.NaN, Double.NaN);

    public final double x, y, z;

    public Cartesian3D(double x, double y, double z)
    {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    @Override
    public Cartesian3D plus(Cartesian3D rhs) {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public Cartesian3D minus(Cartesian3D rhs) {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public Cartesian3D scalarMultiply(double scalar) {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public Cartesian3D scalarDivide(double divisor) {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public Cartesian3D negate() {
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
    public Cartesian3D unitVector() {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public double dot(Cartesian3D rhs) {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public Cartesian3D projectOnto(Cartesian3D otherVec) {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public Cartesian3D cross(Cartesian3D rhs) {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public int getDimension() {
        // TODO Auto-generated method stub
        return 0;
    }
}
