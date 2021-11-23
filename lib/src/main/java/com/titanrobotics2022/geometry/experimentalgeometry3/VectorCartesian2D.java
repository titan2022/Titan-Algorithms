package com.titanrobotics2022.geometry.experimentalgeometry3;

import org.apache.commons.math3.util.FastMath;

public class VectorCartesian2D implements Vector2DOperations<VectorCartesian2D>, CoordinateSystem{

    /** In the Caresian 2D space. */
    private static final CoordinateSystem coordinates = Cartesian2D.getInstance();

    /** Origin (coordinates: 0, 0). */
    public static final VectorCartesian2D ZERO = new VectorCartesian2D(0, 0);

    /** A vector with all coordinates set to NaN. */
    public static final VectorCartesian2D NAN = new VectorCartesian2D(Double.NaN, Double.NaN);

    /** A vector with all coordinates set to positive infinity. */
    public static final VectorCartesian2D POSITIVE_INFINITY = new VectorCartesian2D(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);

    /** A vector with all coordinates set to negative infinity. */
    public static final VectorCartesian2D NEGATIVE_INFINITY = new VectorCartesian2D(Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY);

    public final double x, y;

    public VectorCartesian2D(double x, double y)
    {
        this.x = x;
        this.y = y;
    }

    @Override
    public VectorCartesian2D getZero() {
        return ZERO;
    }

    @Override
    public VectorCartesian2D plus(VectorCartesian2D rhs) {
        return new VectorCartesian2D(x + rhs.x, y + rhs.y);
    }

    @Override
    public VectorCartesian2D minus(VectorCartesian2D rhs) {
        return new VectorCartesian2D(x - rhs.x, y - rhs.y);
    }

    @Override
    public VectorCartesian2D scalarMultiply(double scalar) {
        return new VectorCartesian2D(x * scalar, y * scalar);
    }

    @Override
    public VectorCartesian2D negate() {
        return new VectorCartesian2D(-x, -y);
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
    public VectorCartesian2D unitVector() {
        double mag2 = magnitudeSquared();
        if (mag2 == 0)
            return ZERO;
        else
            return scalarMultiply(1.0 / FastMath.sqrt(mag2));
    }

    @Override
    public double dot(VectorCartesian2D rhs) {
        return x * rhs.x + y * rhs.y;
    }

    @Override
    public VectorCartesian2D projectOnto(VectorCartesian2D otherVec) {
        VectorCartesian2D unitVec = otherVec.unitVector();
        return unitVec.scalarMultiply(dot(unitVec));
    }

    @Override
    public boolean isNaN() {
        return Double.isNaN(x) || Double.isNaN(y);
    }

    @Override
    public boolean isInfinite() {
        return Double.isInfinite(x) || Double.isInfinite(y);
    }

    @Override
	public CoordinateSystem getSpace() {
		return coordinates;
	}

    @Override
    public double cross(VectorCartesian2D rhs) {
        VectorCartesian2D rhs2 = (VectorCartesian2D) rhs;
        return x * rhs2.y -  y * rhs2.x;
    }

    @Override
    public int getDimension() {
        return coordinates.getDimension();
    }

    public static final class Cartesian2D implements CoordinateSystem
    {
        private static Cartesian2D instance = new Cartesian2D();

        private Cartesian2D(){}

        public static Cartesian2D getInstance()
        {
            return instance;
        }

        @Override
        public int getDimension() {
            return 2;
        }
    }
}
