package com.titanrobotics2022.geometry.experimentalgeometry;

import org.apache.commons.math3.util.FastMath;

public class Cartesian2D implements CoordinateSystem, Vector2DOperation<Cartesian2D>, CoordinateTransformation<Polar> {

    /** Origin (coordinates: 0, 0). */
    public static final Cartesian2D ZERO = new Cartesian2D(0, 0);

    /** A vector with all coordinates set to NaN. */
    public static final Cartesian2D NAN = new Cartesian2D(Double.NaN, Double.NaN);

    /** A vector with all coordinates set to positive infinity. */
    public static final Cartesian2D POSITIVE_INFINITY = new Cartesian2D(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);

    /** A vector with all coordinates set to negative infinity. */
    public static final Cartesian2D NEGATIVE_INFINITY = new Cartesian2D(Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY);

    public final double x, y;

    public Cartesian2D(double x, double y)
    {
        this.x = x;
        this.y = y;
    }

    @Override
    public int getDimension() {
        return 2;
    }

    @Override
    public Cartesian2D plus(Cartesian2D rhs) {
        return new Cartesian2D(x + rhs.x, y + rhs.y);
    }

    @Override
    public Cartesian2D minus(Cartesian2D rhs) {
        return new Cartesian2D(x - rhs.x, y - rhs.y);
    }

    @Override
    public Cartesian2D scalarMultiply(double scalar) {
        return new Cartesian2D(x * scalar, y * scalar);
    }

    @Override
    public Cartesian2D scalarDivide(double divisor) {
        double scalar = 1.0 / divisor;
        return new Cartesian2D(x * scalar, y * scalar);
    }

    @Override
    public Cartesian2D negate() {
        return new Cartesian2D(-x, -y);
    }

    @Override
    public double magnitude() {
        return FastMath.sqrt(x * x + y * y);
    }

    @Override
    public double magnitudeSquared() {
        return x * x + y * y;
    }

    @Override
    public Cartesian2D unitVector() {
        double mag2 = magnitudeSquared();
        if (mag2 == 0)
            return ZERO;
        else
            return scalarDivide(FastMath.sqrt(mag2));
    }

    @Override
    public double dot(Cartesian2D rhs) {
        return x * rhs.x + y * rhs.y;
    }

    @Override
    public Cartesian2D projectOnto(Cartesian2D otherVec) {
        Cartesian2D unitVec = otherVec.unitVector();
        return unitVec.scalarMultiply(dot(unitVec));
    }

    @Override //TODO: Verify atan range and cases
    public Polar convertTo(Class<Polar> coordinateSystem) {
        return new Polar(FastMath.atan(y / x), magnitude());
    }

    @Override
    public double cross(Cartesian2D rhs) {
        return x * rhs.y -  y * rhs.x;
    }
}
