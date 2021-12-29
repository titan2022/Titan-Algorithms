package com.titanrobotics2022.geometry.geometry2d;

import com.titanrobotics2022.geometry.CoordinateSystem;
import com.titanrobotics2022.geometry.Vector2DOperations;

import org.apache.commons.math3.util.FastMath;

/**
 *  A Polar geometric vector of the ordered coordinates (r, theta (radians)).
 *  The pole is at 0 theta (radians).
 *  This Polar vector is bijective (injective and surjective) such that there is a unique
 *  Polar vector for each point in 2D space.
 */
public class VectorPolar implements Vector2DOperations<VectorPolar>, CoordinateSystem{
    /** In Polar space. */
    private static final CoordinateSystem coordinates = Polar.getInstance();

    /** Origin (coordinates: (0, 0)). */
    public static final VectorPolar ZERO = new VectorPolar(0, 0);

    /** A vector with all coordinates set to NaN. */
    public static final VectorPolar NAN = new VectorPolar(Double.NaN, Double.NaN);

    /** A vector with all coordinates set to positive infinity. */
    public static final VectorPolar POSITIVE_INFINITY = new VectorPolar(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);

    /** A vector with all coordinates set to negative infinity. */
    public static final VectorPolar NEGATIVE_INFINITY = new VectorPolar(Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY);

    /**
     *  Radius is the distance from the pole.
     *  Domain: [0, inf)
     *  @implNote r is positive as set will be bijective (https://math.stackexchange.com/a/1737911)
     */
    public final double r;
    
    /** 
     *  Angle in radians from Polar axis (equivalent to positive x axis).
     *  Domain: [0, 2PI)
     */
    public final double theta;

    /**
     *  Temporary cartesian version of the Polar vector
     *  @implNote be removed once alternative methods for
     *  all computations are found.
     */
    private final VectorCartesian2D lhsCartesian;

    /**
     *  Constructs a Polar vector with ordered coordinates (r, theta (radians)).
     *  The Polar coordinates pole is along the positive x axis.
     *  <p>Precondition: radius >= 0
     *  <p>Precondition: theta: [0, 2PI)
     *  @param r The radius.
     *  @param theta Polar angle counter clockwise from the pole.
     */
    public VectorPolar(double r, double theta)
    {
        this.r = r;
        this.theta = theta;
        this.lhsCartesian = toCartesian2D();
    }

    // Can be used to speed up initialization of vectors for lhsCartesian
    // private VectorPolar(double r, double theta, VectorCartesian2D lhsCartesian)
    // {
    //     this.r = r;
    //     this.theta = theta;
    //     this.lhsCartesian = lhsCartesian;
    // }

    /**
     *  {@inheritDoc}
     *  Polar coordinates have no origin so (0,0) is a useful origin along the pole.
     *  @return The zero vector (0,0).
     */
	@Override
	public VectorPolar getZero() {
		return ZERO;
	}

    /**
     *  {@inheritDoc}
     */
	@Override
	public VectorPolar plus(VectorPolar rhs) {
		VectorCartesian2D rhsCartesian = rhs.lhsCartesian;
        return this.lhsCartesian.plus(rhsCartesian).toPolar();
	}

    /**
     *  {@inheritDoc}
     */
	@Override
	public VectorPolar minus(VectorPolar rhs) {
		VectorCartesian2D rhsCartesian = rhs.lhsCartesian;
        return this.lhsCartesian.minus(rhsCartesian).toPolar();
	}

    /**
     *  {@inheritDoc}
     */
	@Override
	public VectorPolar scalarMultiply(double scalar) {
		return new VectorPolar(r * scalar, theta);
	}

    /**
     *  {@inheritDoc}
     */
	@Override
	public VectorPolar negate() {
		return new VectorPolar(r, (theta + Math.PI) % (2 * Math.PI));// Does not change r
	}

    /**
     *  {@inheritDoc}
     */
	@Override
	public double magnitude() {
		return FastMath.abs(r);
	}

    /**
     *  {@inheritDoc}
     */
	@Override
	public double magnitudeSquared() {
		return r * r;
	}

    /**
     *  {@inheritDoc}
     */
	@Override
	public VectorPolar unitVector() {
        if (r == 0)
            return ZERO;
        else
            return new VectorPolar(FastMath.copySign(1, r), theta);
	}

    /**
     *  {@inheritDoc}
     *  Polar coordinates are non-linear so the dot product is the dot product by its cartesian representation.
     */
	@Override
	public double dot(VectorPolar rhs) {
		VectorCartesian2D rhsCartesian = rhs.lhsCartesian;
        return this.lhsCartesian.dot(rhsCartesian);
	}

    /**
     *  {@inheritDoc}
     *  Polar coordinates are non-linear so projections are geometric projections.
     */
	@Override
	public VectorPolar projectOnto(VectorPolar targetVec) {
		VectorCartesian2D rhsCartesian = targetVec.lhsCartesian;
        return this.lhsCartesian.projectOnto(rhsCartesian).toPolar();
	}

    /**
     *  {@inheritDoc}
     */
    @Override
    public double azimuthalAngle() {
        return this.lhsCartesian.azimuthalAngle();
    }

    /**
     *  {@inheritDoc}
     */
	@Override
	public boolean isNaN() {
		return Double.isNaN(r) || Double.isNaN(theta);
	}

    /**
     *  {@inheritDoc}
     */
	@Override
	public boolean isInfinite() {
		return Double.isInfinite(r) || Double.isInfinite(theta);
	}

    /**
     *  {@inheritDoc}
     *  The coordinate system is Polar with pole at 0 theta.
     *  Theta is in radians and r >= 0: (r, theta).
     */
	@Override
	public CoordinateSystem getSpace() {
		return coordinates;
	}

    /**
     *  {@inheritDoc}
     *  @implNote Tolerance is from cartesian representation: |lhs - rhs| < tolerance element wise.
     */
    @Override
    public boolean equals(VectorPolar rhs, double tolerance)
    {   
        VectorCartesian2D rhsCartesian = rhs.lhsCartesian;
        return lhsCartesian.equals(rhsCartesian, tolerance);
    }

    /**
     *  {@inheritDoc}
     */
    @Override
	public double cross(VectorPolar rhs) {
		VectorCartesian2D rhsCartesian = rhs.lhsCartesian;
        return this.lhsCartesian.cross(rhsCartesian);
	}

    /**
     *  Converts a Polar vector into a Cartesian 2D vector (x,y).
     *  @return The Cartesian representation of the Polar vector.
     */
    public VectorCartesian2D toCartesian2D()
    {
        if(r == 0)
            return VectorCartesian2D.ZERO;
        else
        {
            double x = r * FastMath.cos(theta);
            double y = r * FastMath.sin(theta);
            return new VectorCartesian2D(x, y);
        }
    }

    /**
     *  {@inheritDoc}
     */
	@Override
	public int getDimension() {
		return coordinates.getDimension();
	}

    /**
     *  A singleton class representing the Polar coordinate system.
     *  Coordinates are (r, theta).
     *  The pole is at 0 theta.
     *  r is the radius, Domain: [0, inf).
     *  theta is the polar angle, Domain: [0, 2PI).
     */
    public static final class Polar implements CoordinateSystem
    {
        private final static Polar instance = new Polar();

        private Polar(){}

        public static Polar getInstance()
        {
            return instance;
        }

        @Override
        public int getDimension() {
            return 2;
        }
    }
}
