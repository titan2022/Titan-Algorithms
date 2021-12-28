package com.titanrobotics2022.geometry.geometry3d;

import com.titanrobotics2022.geometry.CoordinateSystem;
import com.titanrobotics2022.geometry.Vector3DOperations;

import org.apache.commons.math3.util.FastMath;

public class VectorSpherical implements Vector3DOperations<VectorSpherical>, CoordinateSystem{

    /** In the Cartesian 3D space. */
    private static final CoordinateSystem coordinates = Spherical.getInstance();
    
    /** Origin (coordinates: (0, 0, 0)). */
    public static final VectorSpherical ZERO = new VectorSpherical(0, 0, 0);

    /** A vector with all coordinates set to NaN. */
    public static final VectorSpherical NAN = new VectorSpherical(Double.NaN, Double.NaN, Double.NaN);

    /** A vector with all coordinates set to positive infinity. */
    public static final VectorSpherical POSITIVE_INFINITY = new VectorSpherical(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);

    /** A vector with all coordinates set to negative infinity. */
    public static final VectorSpherical NEGATIVE_INFINITY = new VectorSpherical(Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY);
    
    /** The rho component.
     *  Domain: (0, inf)
     *  @implNote rho is positive as set will be bijective
     *  similar to polar coordinates (https://math.stackexchange.com/a/1737911)
     */
    public final double rho;

    /** The theta component.
     *  Domain: [0, 2PI)
     */
    public final double theta;

    /** The phi component.
     *  Domain: [0, PI]
     */
    public final double phi;

    /**
     * Temporary cartesian version of the spherical vector
     * @implNote be removed once alternative methods for
     * computations are found.
     */
    private final VectorCartesian3D lhsCartesian;

    /**
     * Creates a 3D cartesian vector
     * @param rho
     * @param theta
     * @param phi
     */
    public VectorSpherical(double rho, double theta, double phi)
    {
        this.rho = rho;
        this.theta = theta;
        this.phi = phi;
        this.lhsCartesian = this.toCartesian3D();
    }

    @Override
    public VectorSpherical getZero() {
        return ZERO;
    }

    @Override
    public VectorSpherical plus(VectorSpherical rhs) {
        VectorCartesian3D rhsCartesian = rhs.lhsCartesian;
        return this.lhsCartesian.plus(rhsCartesian).toSpherical();
    }

    @Override
    public VectorSpherical minus(VectorSpherical rhs) {
        VectorCartesian3D rhsCartesian = rhs.lhsCartesian;
        return this.lhsCartesian.minus(rhsCartesian).toSpherical();
    }

    @Override
    public VectorSpherical scalarMultiply(double scalar) {
        return this.lhsCartesian.scalarMultiply(scalar).toSpherical();
    }

    @Override
    public VectorSpherical negate() {
        return this.lhsCartesian.negate().toSpherical();
    }

    @Override
    public double magnitude() {
        return FastMath.abs(rho);
    }

    @Override
    public double magnitudeSquared() {
        return rho * rho;
    }

    @Override
    public VectorSpherical unitVector() {
        double mag2 = magnitudeSquared();
        if (mag2 == 0)
            return ZERO;
        else
            return scalarMultiply(Math.copySign(1, rho) / FastMath.sqrt(mag2));
    }

    @Override
    public double dot(VectorSpherical rhs) {
        VectorCartesian3D rhsCartesian = rhs.lhsCartesian;
        return this.lhsCartesian.dot(rhsCartesian);
    }

    @Override
    public VectorSpherical projectOnto(VectorSpherical targetVec) {
        VectorCartesian3D rhsCartesian = targetVec.lhsCartesian;
        return this.lhsCartesian.projectOnto(rhsCartesian).toSpherical();
    }

    @Override
    public double azimuthalAngle() {
        return this.lhsCartesian.azimuthalAngle();
    }

    @Override
    public boolean isNaN() {
        return Double.isNaN(rho) || Double.isNaN(theta) || Double.isNaN(phi);
    }

    @Override
    public boolean isInfinite() {
        return Double.isInfinite(rho) || Double.isInfinite(theta) || Double.isInfinite(phi);
    }

    @Override
    public CoordinateSystem getSpace() {
        return coordinates;
    }

    @Override
    public boolean equals(VectorSpherical rhs, double tolerance) {
        return FastMath.abs(rho - rhs.rho) < tolerance
            && FastMath.abs(theta - rhs.theta) < tolerance 
            && FastMath.abs(phi - rhs.phi) < tolerance;
    }

    @Override
    public int getDimension() {
        return coordinates.getDimension();
    }

    @Override
    public VectorSpherical cross(VectorSpherical rhs) {
        VectorCartesian3D rhsCartesian = rhs.lhsCartesian;
        return this.lhsCartesian.cross(rhsCartesian).toSpherical();
    }

    public VectorCartesian3D toCartesian3D()
    {
        double x = rho * FastMath.sin(phi) * FastMath.cos(theta);
        double y = rho * FastMath.sin(phi) * FastMath.sin(theta);
        double z = rho * FastMath.cos(phi);
        return new VectorCartesian3D(x, y, z);
    }

    public VectorCylindrical toCylindrical()
    {
        double r = rho * FastMath.sin(phi);
        double z = rho * FastMath.cos(phi);
        return new VectorCylindrical(r, theta, z);
    }
    
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
