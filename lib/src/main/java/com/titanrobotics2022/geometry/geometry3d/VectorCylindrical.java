package com.titanrobotics2022.geometry.geometry3d;

import com.titanrobotics2022.geometry.CoordinateSystem;
import com.titanrobotics2022.geometry.Vector3DOperations;

import org.apache.commons.math3.util.FastMath;

/**
 *  A Cylindrical geometric vector of the ordered coordinates (r, theta (radians), z).
 *  The pole is at 0 theta (radians).
 *  This Cylindrical vector is bijective (injective and surjective) such that there is a unique
 *  Cylindrical vector for each point in 3D space.
 *  @see https://tutorial.math.lamar.edu/Classes/CalcII/CylindricalCoords.aspx
 */
public class VectorCylindrical implements Vector3DOperations<VectorCylindrical>, CoordinateSystem{

    /** In Cylindrical space. */
    private static final CoordinateSystem coordinates = Cylindrical.getInstance();
    
    /** Origin (coordinates: (0, 0, 0)). */
    public static final VectorCylindrical ZERO = new VectorCylindrical(0, 0, 0);

    /** A vector with all coordinates set to NaN. */
    public static final VectorCylindrical NAN = new VectorCylindrical(Double.NaN, Double.NaN, Double.NaN);

    /** A vector with all coordinates set to positive infinity. */
    public static final VectorCylindrical POSITIVE_INFINITY = new VectorCylindrical(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);

    /** A vector with all coordinates set to negative infinity. */
    public static final VectorCylindrical NEGATIVE_INFINITY = new VectorCylindrical(Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY);
    
    /**
     *  Radius is the distance from the pole.
     *  Domain: [0, inf)
     *  @implNote r is positive as set will be bijective (https://math.stackexchange.com/a/1737911)
     */
    public final double r;

    /** 
     *  Angle in radians from pole (equivalent to positive x axis).
     *  Domain: [0, 2PI)
     */
    public final double theta;

    /** The z component. (-inf, inf)*/
    public final double z;

    /**
     *  Temporary cartesian version of the cylindrical vector
     *  @implNote be removed once alternative methods for
     *  computations are found.
     */
    private final VectorCartesian3D lhsCartesian;

    /**
     *  Constructs a Cylindrical vector with ordered coordinates (r, theta (radians), z).
     *  The theta pole is along the positive x axis.
     *  <p>Precondition: radius >= 0
     *  <p>Precondition: theta: [0, 2PI)
     *  @param r The radius.
     *  @param theta Polar angle counter clockwise from the pole.
     *  @param z The vertical direction variable.
     */
    public VectorCylindrical(double r, double theta, double z)
    {
        this.r = r;
        this.theta = theta;
        this.z = z;
        this.lhsCartesian = this.toCartesian3D();
    }

    /**
     *  {@inheritDoc}
     *  Cylindrical coordinates have no exact origin so (0,0,0) is a useful origin along the pole.
     *  @return The zero vector (0,0,0).
     */
    @Override
    public VectorCylindrical getZero() {
        return ZERO;
    }

    /**
     *  {@inheritDoc}
     */
    @Override
    public VectorCylindrical plus(VectorCylindrical rhs) {
        VectorCartesian3D rhsCartesian = rhs.lhsCartesian;
        return this.lhsCartesian.plus(rhsCartesian).toCylindrical();
    }

    /**
     *  {@inheritDoc}
     */
    @Override
    public VectorCylindrical minus(VectorCylindrical rhs) {
        VectorCartesian3D rhsCartesian = rhs.lhsCartesian;
        return this.lhsCartesian.minus(rhsCartesian).toCylindrical();
    }

    /**
     *  {@inheritDoc}
     */
    @Override
    public VectorCylindrical scalarMultiply(double scalar) {
        return new VectorCylindrical(r * scalar, theta, z * scalar);
    }

    /**
     *  {@inheritDoc}
     */
    @Override
    public VectorCylindrical negate() {
        return new VectorCylindrical(r, (theta + Math.PI) % (2 * Math.PI), -z);
    }

    /**
     *  {@inheritDoc}
     */
    @Override
    public double magnitude() {
        return FastMath.sqrt(r * r + z * z);
    }

    /**
     *  {@inheritDoc}
     */
    @Override
    public double magnitudeSquared() {
        return r * r + z * z;
    }

    /**
     *  {@inheritDoc}
     */
    @Override
    public VectorCylindrical unitVector() {
        double mag2 = magnitudeSquared();
        if (mag2 == 0)
            return ZERO;
        else
            return scalarMultiply(1 / FastMath.sqrt(mag2));
    }

    /**
     *  {@inheritDoc}
     *  Cylindrical coordinates are non-linear so the dot product is the dot product by its geoemtric cartesian representation.
     */
    @Override
    public double dot(VectorCylindrical rhs) {
        VectorCartesian3D rhsCartesian = rhs.lhsCartesian;
        return this.lhsCartesian.dot(rhsCartesian);
    }

    /**
     *  {@inheritDoc}
     *  Cylindrical coordinates are non-linear so the projection is by its geoemtric cartesian representation.
     */
    @Override
    public VectorCylindrical projectOnto(VectorCylindrical targetVec) {
        VectorCartesian3D rhsCartesian = targetVec.lhsCartesian;
        return this.lhsCartesian.projectOnto(rhsCartesian).toCylindrical();
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
        return Double.isNaN(r) || Double.isNaN(theta) || Double.isNaN(z);
    }

    /**
     *  {@inheritDoc}
     */
    @Override
    public boolean isInfinite() {
        return Double.isInfinite(r) || Double.isInfinite(theta) || Double.isInfinite(z);
    }

    /**
     *  {@inheritDoc}
     *  The coordinate system is Cylindrical with pole at 0 theta.
     *  Theta is in radians, and r >= 0: (r, theta).
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
    public boolean equals(VectorCylindrical rhs, double tolerance) {
        return FastMath.abs(r - rhs.r) < tolerance
            && FastMath.abs(theta - rhs.theta) < tolerance 
            && FastMath.abs(z - rhs.z) < tolerance;
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
    public VectorCylindrical cross(VectorCylindrical rhs) {
        VectorCartesian3D rhsCartesian = rhs.lhsCartesian;
        return this.lhsCartesian.cross(rhsCartesian).toCylindrical();
    }

    /**
     *  Converts a Cylindrical vector into a Cartesian 3D vector (x,y,z).
     *  @return The Cartesian representation of the Cylindrical vector.
     */
    public VectorCartesian3D toCartesian3D()
    {
        double x = r * FastMath.cos(theta);
        double y = r * FastMath.sin(theta);
        return new VectorCartesian3D(x, y, z);
    }

    /**
     *  Converts a Cylindrical vector into a Spherical vector (rho, theta (radians), phi (radians)).
     *  @return The Spherical representation of the Cylindrical vector.
     *  @implNote theta is maintained. 
     *  @see https://math.libretexts.org/Bookshelves/Calculus/Book%3A_Calculus_(OpenStax)/12%3A_Vectors_in_Space/12.7%3A_Cylindrical_and_Spherical_Coordinates.
     */
    public VectorSpherical toSpherical()
    {
        double rho = FastMath.sqrt(r * r + z * z);
        double phi = FastMath.acos(z / rho);
        return new VectorSpherical(rho, theta, phi);
    }
    
    /**
     *  A singleton class representing the Cylindrical coordinate system.
     *  Coordinates are (r, theta, z).
     *  The pole is at 0 theta.
     *  r is the radius, Domain: [0, inf).
     *  theta is the polar angle, Domain: [0, 2PI).
     *  z is the vertical coordinate, Domain: (-inf, inf).
     */
    public static final class Cylindrical implements CoordinateSystem
    {
        private final static Cylindrical instance = new Cylindrical();

        private Cylindrical(){}

        public static Cylindrical getInstance()
        {
            return instance;
        }

        @Override
        public int getDimension() {
            return 3;
        }
    }
}
