package com.titanrobotics2022.geometry.geometry3d;

import com.titanrobotics2022.geometry.CoordinateSystem;
import com.titanrobotics2022.geometry.Vector3DOperations;

import org.apache.commons.math3.util.FastMath;

/**
 *  A Spherical geometric vector of the ordered coordinates (rho, theta (radians), phi).
 *  The pole is at 0 theta (radians).
 *  This Spherical vector is bijective (injective and surjective) such that there is a unique
 *  Spherical vector for each point in 3D space.
 */
public class VectorSpherical implements Vector3DOperations<VectorSpherical>, CoordinateSystem{

    /** In Spherical space. */
    private static final CoordinateSystem coordinates = Spherical.getInstance();
    
    /** Origin (coordinates: (0, 0, 0)). */
    public static final VectorSpherical ZERO = new VectorSpherical(0, 0, 0);

    /** A vector with all coordinates set to NaN. */
    public static final VectorSpherical NAN = new VectorSpherical(Double.NaN, Double.NaN, Double.NaN);

    /** A vector with all coordinates set to positive infinity. */
    public static final VectorSpherical POSITIVE_INFINITY = new VectorSpherical(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);

    /** A vector with all coordinates set to negative infinity. */
    public static final VectorSpherical NEGATIVE_INFINITY = new VectorSpherical(Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY);
    
    /** The rho component, distance of point from origin.
     *  Domain: [0, inf)
     *  @implNote rho is positive as set will be bijective
     *  similar to polar coordinates (https://math.stackexchange.com/a/1737911).
     */
    public final double rho;

    /** 
     *  Angle in radians from pole (equivalent to positive x axis).
     *  Domain: [0, 2PI)
     */
    public final double theta;

    /** 
     *  The phi component.
     *  Domain: [0, PI]
     */
    public final double phi;

    /**
     *  Temporary cartesian version of the spherical vector
     *  @implNote be removed once alternative methods for
     *  computations are found.
     */
    private final VectorCartesian3D lhsCartesian;

    /**
     *  Constructs a Spherical vector with ordered coordinates (rho, theta (radians), phi (radians)).
     *  The theta pole is along the positive x axis.
     *  <p>Precondition: rho >= 0
     *  <p>Precondition: theta: [0, 2PI)
     *  @param rho The distance from origin.
     *  @param theta Angle counter clockwise from the pole in radians.
     *  @param phi The vertical direction variable.
     */
    public VectorSpherical(double rho, double theta, double phi)
    {
        this.rho = rho;
        this.theta = theta;
        this.phi = phi;
        this.lhsCartesian = this.toCartesian3D();
    }

    /**
     *  {@inheritDoc}
     *  Spherical coordinates have no exact origin so (0,0,0) is a useful but arbitrary origin along the pole.
     *  @return The zero vector (0,0,0).
     */
    @Override
    public VectorSpherical getZero() {
        return ZERO;
    }

    /**
     *  {@inheritDoc}
     */
    @Override
    public VectorSpherical plus(VectorSpherical rhs) {
        VectorCartesian3D rhsCartesian = rhs.lhsCartesian;
        return this.lhsCartesian.plus(rhsCartesian).toSpherical();
    }

    /**
     *  {@inheritDoc}
     */
    @Override
    public VectorSpherical minus(VectorSpherical rhs) {
        VectorCartesian3D rhsCartesian = rhs.lhsCartesian;
        return this.lhsCartesian.minus(rhsCartesian).toSpherical();
    }

    /**
     *  {@inheritDoc}
     */
    @Override
    public VectorSpherical scalarMultiply(double scalar) {
        return this.lhsCartesian.scalarMultiply(scalar).toSpherical();
    }

    /**
     *  {@inheritDoc}
     */
    @Override
    public VectorSpherical negate() {
        return this.lhsCartesian.negate().toSpherical();
    }

    /**
     *  {@inheritDoc}
     */
    @Override
    public double magnitude() {
        return FastMath.abs(rho);
    }

    /**
     *  {@inheritDoc}
     */
    @Override
    public double magnitudeSquared() {
        return rho * rho;
    }

    /**
     *  {@inheritDoc}
     */
    @Override
    public VectorSpherical unitVector() {
        double mag2 = magnitudeSquared();
        if (mag2 == 0)
            return ZERO;
        else
            return scalarMultiply(Math.copySign(1, rho) / FastMath.sqrt(mag2));
    }

    /**
     *  {@inheritDoc}
     *  Spherical coordinates are non-linear so the dot product is the dot product by its geoemtric cartesian representation.
     */
    @Override
    public double dot(VectorSpherical rhs) {
        VectorCartesian3D rhsCartesian = rhs.lhsCartesian;
        return this.lhsCartesian.dot(rhsCartesian);
    }

    /**
     *  {@inheritDoc}
     *  Spherical coordinates are non-linear so the projection is by its geoemtric cartesian representation.
     */
    @Override
    public VectorSpherical projectOnto(VectorSpherical targetVec) {
        VectorCartesian3D rhsCartesian = targetVec.lhsCartesian;
        return this.lhsCartesian.projectOnto(rhsCartesian).toSpherical();
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
        return Double.isNaN(rho) || Double.isNaN(theta) || Double.isNaN(phi);
    }

    /**
     *  {@inheritDoc}
     */
    @Override
    public boolean isInfinite() {
        return Double.isInfinite(rho) || Double.isInfinite(theta) || Double.isInfinite(phi);
    }

    /**
     *  {@inheritDoc}
     *  The coordinate system is Spherical with pole at 0 theta.
     *  Theta and phi are in radians, and rho >= 0: (rho, theta, phi).
     */
    @Override
    public CoordinateSystem getSpace() {
        return coordinates;
    }

    /**
     *  {@inheritDoc}
     *  @implNote Tolerance is from unnormalized vector difference: |lhs - rhs| < tolerance element wise.
     */
    @Override
    public boolean equals(VectorSpherical rhs, double tolerance) {
        return FastMath.abs(rho - rhs.rho) < tolerance
            && FastMath.abs(theta - rhs.theta) < tolerance 
            && FastMath.abs(phi - rhs.phi) < tolerance;
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
    public VectorSpherical cross(VectorSpherical rhs) {
        VectorCartesian3D rhsCartesian = rhs.lhsCartesian;
        return this.lhsCartesian.cross(rhsCartesian).toSpherical();
    }

    /**
     *  Converts a Spherical vector into a Cartesian 3D vector (x,y,z).
     *  @return The Cartesian representation of the Spherical vector.
     */
    public VectorCartesian3D toCartesian3D()
    {
        double x = rho * FastMath.sin(phi) * FastMath.cos(theta);
        double y = rho * FastMath.sin(phi) * FastMath.sin(theta);
        double z = rho * FastMath.cos(phi);
        return new VectorCartesian3D(x, y, z);
    }

    /**
     *  Converts a Spherical vector into a Cylindrical vector (r, theta (radians), z).
     *  @return The Spherical representation of the Cylindrical vector.
     *  @implNote theta is maintained. 
     *  @see https://math.libretexts.org/Bookshelves/Calculus/Book%3A_Calculus_(OpenStax)/12%3A_Vectors_in_Space/12.7%3A_Cylindrical_and_Spherical_Coordinates.
     */
    public VectorCylindrical toCylindrical()
    {
        double r = rho * FastMath.sin(phi);
        double z = rho * FastMath.cos(phi);
        return new VectorCylindrical(r, theta, z);
    }
    
    /**
     *  A singleton class representing the Spherical coordinate system.
     *  Coordinates are (rho, theta, phi).
     *  The pole is at 0 theta.
     *  rho is the distance from the origin, Domain: [0, inf).
     *  theta is the polar angle, Domain: [0, 2PI).
     *  phi is the descention angle, Domain: [0, PI].
     */
    public static final class Spherical implements CoordinateSystem
    {
        private final static Spherical instance = new Spherical();

        private Spherical(){}

        public static Spherical getInstance()
        {
            return instance;
        }

        @Override
        public int getDimension() {
            return 3;
        }
    }
}
