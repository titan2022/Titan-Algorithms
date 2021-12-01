package com.titanrobotics2022.geometry.geometry3d;

import com.titanrobotics2022.geometry.CoordinateSystem;
import com.titanrobotics2022.geometry.Vector3DOperations;

import org.apache.commons.math3.util.FastMath;

public class VectorCylindrical implements Vector3DOperations<VectorCylindrical>, CoordinateSystem{

    /** In the Cartesian 3D space. */
    private static final CoordinateSystem coordinates = Cylindrical.getInstance();
    
    /** Origin (coordinates: (0, 0, 0)). */
    public static final VectorCylindrical ZERO = new VectorCylindrical(0, 0, 0);

    /** A vector with all coordinates set to NaN. */
    public static final VectorCylindrical NAN = new VectorCylindrical(Double.NaN, Double.NaN, Double.NaN);

    /** A vector with all coordinates set to positive infinity. */
    public static final VectorCylindrical POSITIVE_INFINITY = new VectorCylindrical(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);

    /** A vector with all coordinates set to negative infinity. */
    public static final VectorCylindrical NEGATIVE_INFINITY = new VectorCylindrical(Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY);
    
    /** The r component. (-inf, inf)*/
    public final double r;

    /** The theta component. [0,2PI)*/
    public final double theta;

    /** The z component. (-inf, inf)*/
    public final double z;

    /**
     * Temporary cartesian version of the cylindrical vector
     * @implNote be removed once alternative methods for
     * computations are found.
     */
    private final VectorCartesian3D lhsCartesian;

    /**
     * Creates a 3D cartesian vector
     * @param r
     * @param theta
     * @param z
     */
    public VectorCylindrical(double r, double theta, double z)
    {
        this.r = r;
        this.theta = theta;
        this.z = z;
        this.lhsCartesian = this.toCartesian3D();
    }

    @Override
    public VectorCylindrical getZero() {
        return ZERO;
    }

    @Override
    public VectorCylindrical plus(VectorCylindrical rhs) {
        VectorCartesian3D rhsCartesian = rhs.lhsCartesian;
        return this.lhsCartesian.plus(rhsCartesian).toCylindrical();
    }

    @Override
    public VectorCylindrical minus(VectorCylindrical rhs) {
        VectorCartesian3D rhsCartesian = rhs.lhsCartesian;
        return this.lhsCartesian.minus(rhsCartesian).toCylindrical();
    }

    @Override
    public VectorCylindrical scalarMultiply(double scalar) {
        return new VectorCylindrical(r * scalar, theta, z * scalar);
    }

    @Override
    public VectorCylindrical negate() {
        return new VectorCylindrical(r, (theta + Math.PI) % (2 * Math.PI), -z);
    }

    @Override
    public double magnitude() {
        return FastMath.sqrt(r * r + z * z);
    }

    @Override
    public double magnitudeSquared() {
        return r * r + z * z;
    }

    @Override
    public VectorCylindrical unitVector() {
        double mag2 = magnitudeSquared();
        if (mag2 == 0)
            return ZERO;
        else
            return scalarMultiply(1 / FastMath.sqrt(mag2));
    }

    @Override
    public double dot(VectorCylindrical rhs) {
        VectorCartesian3D rhsCartesian = rhs.lhsCartesian;
        return this.lhsCartesian.dot(rhsCartesian);
    }

    @Override
    public VectorCylindrical projectOnto(VectorCylindrical targetVec) {
        VectorCartesian3D rhsCartesian = targetVec.lhsCartesian;
        return this.lhsCartesian.projectOnto(rhsCartesian).toCylindrical();
    }

    @Override
    public double azimuthalAngle() {
        return this.lhsCartesian.azimuthalAngle();
    }

    @Override
    public boolean isNaN() {
        return Double.isNaN(r) || Double.isNaN(theta) || Double.isNaN(z);
    }

    @Override
    public boolean isInfinite() {
        return Double.isInfinite(r) || Double.isInfinite(theta) || Double.isInfinite(z);
    }

    @Override
    public CoordinateSystem getSpace() {
        return coordinates;
    }

    @Override
    public boolean equals(VectorCylindrical rhs, double tolerance) {
        return FastMath.abs(r - rhs.r) < tolerance
            && FastMath.abs(theta - rhs.theta) < tolerance 
            && FastMath.abs(z - rhs.z) < tolerance;
    }

    @Override
    public int getDimension() {
        return coordinates.getDimension();
    }

    @Override
    public VectorCylindrical cross(VectorCylindrical rhs) {
        VectorCartesian3D rhsCartesian = rhs.lhsCartesian;
        return this.lhsCartesian.cross(rhsCartesian).toCylindrical();
    }

    public VectorCartesian3D toCartesian3D()
    {
        double x = r * FastMath.cos(theta);
        double y = r * FastMath.sin(theta);
        return new VectorCartesian3D(x, y, z);
    }

    // From: https://math.libretexts.org/Bookshelves/Calculus/Book%3A_Calculus_(OpenStax)/12%3A_Vectors_in_Space/12.7%3A_Cylindrical_and_Spherical_Coordinates
    public VectorSpherical toSpherical()
    {
        double rho = FastMath.sqrt(r * r + z * z);
        double phi = FastMath.acos(z / rho);
        return new VectorSpherical(rho, theta, phi);
    }
    
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
