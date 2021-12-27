package com.titanrobotics2022.geometry.geometry2d;

import com.titanrobotics2022.geometry.CoordinateSystem;
import com.titanrobotics2022.geometry.Vector2DOperations;

import org.apache.commons.math3.util.FastMath;

public class VectorPolar implements Vector2DOperations<VectorPolar>, CoordinateSystem{
    /** In polar space. */
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
    
    /** Angle in radians from polar axis (equivalent to positive x axis).
     *  Domain: [0, 2PI)
     */
    public final double theta;

    /**
     * Temporary cartesian version of the polar vector
     * @implNote be removed once alternative methods for
     * all computations are found.
     */
    private final VectorCartesian2D lhsCartesian;

    /**
     * Constructs a polar coordinate that is in 2 dimensions
     * @param r radius
     * @param theta 
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

	@Override
	public VectorPolar getZero() {
		return ZERO;
	}

	@Override
	public VectorPolar plus(VectorPolar rhs) {
		VectorCartesian2D rhsCartesian = rhs.lhsCartesian;
        return this.lhsCartesian.plus(rhsCartesian).toPolar();
	}

	@Override
	public VectorPolar minus(VectorPolar rhs) {
		VectorCartesian2D rhsCartesian = rhs.lhsCartesian;
        return this.lhsCartesian.minus(rhsCartesian).toPolar();
	}

	@Override
	public VectorPolar scalarMultiply(double scalar) {
		return new VectorPolar(r * scalar, theta);
	}

	@Override
	public VectorPolar negate() {
		return new VectorPolar(r, (theta + Math.PI) % (2 * Math.PI));// Does not change r
	}

	@Override
	public double magnitude() {
		return FastMath.abs(r);
	}

	@Override
	public double magnitudeSquared() {
		return FastMath.sqrt(FastMath.abs(r));
	}

	@Override
	public VectorPolar unitVector() {
        if (r == 0)
            return ZERO;
        else
            return new VectorPolar(FastMath.copySign(1, r), theta);
	}

	@Override
	public double dot(VectorPolar rhs) {
		VectorCartesian2D rhsCartesian = rhs.lhsCartesian;
        return this.lhsCartesian.dot(rhsCartesian);
	}

	@Override
	public VectorPolar projectOnto(VectorPolar targetVec) {
		VectorCartesian2D rhsCartesian = targetVec.lhsCartesian;
        return this.lhsCartesian.projectOnto(rhsCartesian).toPolar();
	}

    @Override
    public double azimuthalAngle() {
        return this.lhsCartesian.azimuthalAngle();
    }

	@Override
	public boolean isNaN() {
		return Double.isNaN(r) || Double.isNaN(theta);
	}

	@Override
	public boolean isInfinite() {
		return Double.isInfinite(r) || Double.isInfinite(theta);
	}

	@Override
	public CoordinateSystem getSpace() {
		return coordinates;
	}

    @Override
    public boolean equals(VectorPolar rhs, double tolerance)
    {   
        VectorCartesian2D rhsCartesian = rhs.lhsCartesian;
        return lhsCartesian.equals(rhsCartesian, tolerance);
    }

    @Override
	public double cross(VectorPolar rhs) {
		VectorCartesian2D rhsCartesian = rhs.lhsCartesian;
        return this.lhsCartesian.cross(rhsCartesian);
	}

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

	@Override
	public int getDimension() {
		return coordinates.getDimension();
	}

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
