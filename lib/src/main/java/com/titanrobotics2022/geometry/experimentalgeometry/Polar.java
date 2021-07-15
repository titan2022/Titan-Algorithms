package com.titanrobotics2022.geometry.experimentalgeometry;

public class Polar implements CoordinateSystem, Vector2DOperation<Polar>, CoordinateTransformation<Cartesian2D>{
    /** Origin (coordinates: 0, 0). */
    public static final Polar ZERO = new Polar(0, 0);

    /** A vector with all coordinates set to NaN. */
    public static final Polar NAN = new Polar(Double.NaN, Double.NaN);

    public final double r, theta;

    public Polar(double r, double theta)
    {
        this.r = r;
        this.theta = theta;
    }

    @Override
    public int getDimension() {
        // TODO Auto-generated method stub
        return 2;
    }

    @Override
    public Polar plus(Polar rhs) {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public Polar minus(Polar rhs) {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public Polar scalarMultiply(double scalar) {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public Polar scalarDivide(double divisor) {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public Polar negate() {
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
    public Polar unitVector() {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public double dot(Polar rhs) {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public Polar projectOnto(Polar otherVec) {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public Cartesian2D convertTo(Class<Cartesian2D> coordinateSystem) {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public double cross(Polar rhs) {
        // TODO Auto-generated method stub
        return 0;
    } 
}
