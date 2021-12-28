package com.titanrobotics2022.geometry.geometry2d;

import com.titanrobotics2022.geometry.CoordinateSystem;
import com.titanrobotics2022.geometry.Vector2DOperations;

import org.apache.commons.math3.util.FastMath;

/**
 *  A cartesian 2D geometric vector of the ordered coordinates (x,y).
 */
public class VectorCartesian2D implements Vector2DOperations<VectorCartesian2D>, CoordinateSystem{

    /** In the Cartesian 2D space. */
    private static final CoordinateSystem coordinates = Cartesian2D.getInstance();

    /** Origin (coordinates: (0, 0)). */
    public static final VectorCartesian2D ZERO = new VectorCartesian2D(0, 0);

    /** A vector with all coordinates set to NaN. */
    public static final VectorCartesian2D NAN = new VectorCartesian2D(Double.NaN, Double.NaN);

    /** A vector with all coordinates set to positive infinity. */
    public static final VectorCartesian2D POSITIVE_INFINITY = new VectorCartesian2D(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);

    /** A vector with all coordinates set to negative infinity. */
    public static final VectorCartesian2D NEGATIVE_INFINITY = new VectorCartesian2D(Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY);

    public final double x, y;

    /**
     *  Constructs a Cartesian 2D vector with ordered coordinates (x,y)
     *  @param x component (abscissa)
     *  @param y component (ordinate)
     */
    public VectorCartesian2D(double x, double y)
    {
        this.x = x;
        this.y = y;
    }

    /**
     *  {@inheritDoc}
     *  Cartesian 2D origin is (0,0)
     *  @return zero vector (0,0)
     */
    @Override
    public VectorCartesian2D getZero() {
        return ZERO;
    }

    /**
     *  {@inheritDoc}
     */
    @Override
    public VectorCartesian2D plus(VectorCartesian2D rhs) {
        return new VectorCartesian2D(x + rhs.x, y + rhs.y);
    }

    /**
     *  {@inheritDoc}
     */
    @Override
    public VectorCartesian2D minus(VectorCartesian2D rhs) {
        return new VectorCartesian2D(x - rhs.x, y - rhs.y);
    }

    /**
     *  {@inheritDoc}
     */
    @Override
    public VectorCartesian2D scalarMultiply(double scalar) {
        return new VectorCartesian2D(x * scalar, y * scalar);
    }

    /**
     *  {@inheritDoc}
     */
    @Override
    public VectorCartesian2D negate() {
        return new VectorCartesian2D(-x, -y);
    }

    /**
     *  {@inheritDoc}
     */
    @Override
    public double magnitude() {
        return FastMath.sqrt(x * x + y * y);
    }

    /**
     *  {@inheritDoc}
     */
    @Override
    public double magnitudeSquared() {
        return x * x + y * y;
    }

    /**
     *  {@inheritDoc}
     */
    @Override
    public VectorCartesian2D unitVector() {
        double mag2 = magnitudeSquared();
        if (mag2 == 0)
            return ZERO;
        else
            return scalarMultiply(1.0 / FastMath.sqrt(mag2));
    }

    /**
     *  {@inheritDoc}
     *  <p>result = sum(ai * bi) from i to 3, i = 0
     */
    @Override
    public double dot(VectorCartesian2D rhs) {
        return x * rhs.x + y * rhs.y;
    }

    /**
     *  {@inheritDoc}
     */
    @Override
    public VectorCartesian2D projectOnto(VectorCartesian2D otherVec) {
        VectorCartesian2D unitVec = otherVec.unitVector();
        return unitVec.scalarMultiply(dot(unitVec));
    }

    /**
     *  {@inheritDoc}
     */
    @Override
    public double azimuthalAngle() {
        double answer = FastMath.atan2(y, x);
        if (answer < 0) // Fixes round off error near 0
            answer += 2 * Math.PI;
        return answer;
    }

    /**
     *  {@inheritDoc}
     */
    @Override
    public boolean isNaN() {
        return Double.isNaN(x) || Double.isNaN(y);
    }

    /**
     *  {@inheritDoc}
     */
    @Override
    public boolean isInfinite() {
        return Double.isInfinite(x) || Double.isInfinite(y);
    }

    /**
     *  {@inheritDoc}
     *  The coordinate system is Cartesian 2D (x,y).
     */
    @Override
	public CoordinateSystem getSpace() {
		return coordinates;
	}

    /**
     *  {@inheritDoc}
     *  @implNote Tolerance is |lhs - rhs| < tolerance element wise.
     */
    @Override
    public boolean equals(VectorCartesian2D rhs, double tolerance) {
        return FastMath.abs(x - rhs.x) < tolerance && FastMath.abs(y - rhs.y) < tolerance;
    }

    /**
     *  {@inheritDoc}
     */
    @Override
    public double cross(VectorCartesian2D rhs) {
        return x * rhs.y -  y * rhs.x;
    }

    /**
     *  {@inheritDoc}
     */
    @Override
    public int getDimension() {
        return coordinates.getDimension();
    }

    /**
     *  {@inheritDoc}
     */
    @Override
    public boolean equals(Object obj)
    {
        if (obj instanceof VectorCartesian2D)
        {
            final VectorCartesian2D rhs = (VectorCartesian2D) obj;
            return (x == rhs.x) && (y == rhs.y);
        }
        return false;
    }

    private final static double delta = 1e-15;

    /**
     *  Converts a Cartesian 2D vector into a Polar vector (r, theta (radians)).
     *  @return The Polar representation of the Cartesian 2D vector.
     *  If the Cartesian 2D vector is (0,0), then the returning
     *  vector will point along the Polar pole (theta = 0 radians). 
     */
    public VectorPolar toPolar()
    {
        double rsquared = magnitudeSquared();
        if(rsquared == 0)
            return VectorPolar.ZERO;
        else
        {
            double theta = FastMath.atan2(y, x);
            if(theta < -delta)
                theta += 2 * Math.PI;
            else if(theta < delta) // Sets theta to be 0 exactly, theta was previous evaluated to be greater than negative delta
                theta = 0;
            return new VectorPolar(FastMath.sqrt(rsquared), theta);
        }        
    }

    /**
     *  Computes the shortest angle between two vectors.
     *  @param v1 Vector 1.
     *  @param v2 Vector 2.
     *  @return Angle in radians.
     */
    public static double shortestAngleBetween(VectorCartesian2D v1, VectorCartesian2D v2)
    {
        double cosAngle = v1.dot(v2) / (v1.magnitude() * v2.magnitude());

        // result can exceed 1 due to roundoff error for nearly identical vectors
        if (cosAngle > 1)
            return 0;
        else if (cosAngle < -1)
            return Math.PI;
        else
            return FastMath.acos(cosAngle);
    }

    /**
     *  A singleton class representing the Cartesian 2D coordinate system.
     *  Coordinates are (x,y)
     *  x is the abscissa
     *  y is the ordinate
     */
    public static final class Cartesian2D implements CoordinateSystem
    {
        private final static Cartesian2D instance = new Cartesian2D();

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
