package com.titanrobotics2022.geometry.experimentalgeometry3;

import org.apache.commons.math3.util.FastMath;

public class VectorCartesian3D implements Vector3DOperations<VectorCartesian3D>, CoordinateSystem{
    
    /** In the Cartesian 3D space. */
    private static final CoordinateSystem coordinates = Cartesian3D.getInstance();
    
    /** Origin (coordinates: (0, 0, 0)). */
    public static final VectorCartesian3D ZERO = new VectorCartesian3D(0, 0, 0);

    /** A vector with all coordinates set to NaN. */
    public static final VectorCartesian3D NAN = new VectorCartesian3D(Double.NaN, Double.NaN, Double.NaN);

    /** A vector with all coordinates set to positive infinity. */
    public static final VectorCartesian3D POSITIVE_INFINITY = new VectorCartesian3D(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);

    /** A vector with all coordinates set to negative infinity. */
    public static final VectorCartesian3D NEGATIVE_INFINITY = new VectorCartesian3D(Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY);
    
    /** The x component. */
    public final double x;

    /** The y component. */
    public final double y;

    /** The z component. */
    public final double z;

    /**
     * Creates a 3D cartesian vector
     * @param x
     * @param y
     * @param z
     */
    public VectorCartesian3D(double x, double y, double z)
    {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    @Override
    public VectorCartesian3D getZero() {
        return ZERO;
    }

    @Override
    public VectorCartesian3D plus(VectorCartesian3D rhs) {
        return new VectorCartesian3D(this.x + rhs.x, this.y + rhs.y, this.z + rhs.z);
    }

    @Override
    public VectorCartesian3D minus(VectorCartesian3D rhs) {
        return new VectorCartesian3D(this.x - rhs.x, this.y - rhs.y, this.z - rhs.z);
    }

    @Override
    public VectorCartesian3D scalarMultiply(double scalar) {
        return new VectorCartesian3D(this.x * scalar, this.y * scalar, this.z * scalar);
    }

    @Override
    public VectorCartesian3D negate() {
        return new VectorCartesian3D(-x, -y, -z);
    }

    @Override
    public double magnitude() {
        return FastMath.sqrt(x * x + y * y + z * z);
    }

    @Override
    public double magnitudeSquared() {
        return x * x + y * y + z * z;
    }

    @Override
    public VectorCartesian3D unitVector() {
        double mag2 = magnitudeSquared();
        if (mag2 == 0)
            return ZERO;
        else
            return scalarMultiply(1.0 / FastMath.sqrt(mag2));
    }

    @Override
    public double dot(VectorCartesian3D rhs) {
        return this.x * rhs.x + this.y * rhs.y + this.z * rhs.z;
    }

    @Override
    public VectorCartesian3D projectOnto(VectorCartesian3D targetVec) {
        VectorCartesian3D unitVec = targetVec.unitVector();
        return unitVec.scalarMultiply(dot(unitVec));
    }

    @Override
    public double azimuthalAngle() {
        double answer = FastMath.atan2(y, x);
        if (answer < 0)
            answer += 2 * Math.PI;
        return answer;
    }

    @Override
    public boolean isNaN() {
        return Double.isNaN(x) || Double.isNaN(y) || Double.isNaN(z);
    }

    @Override
    public boolean isInfinite() {
        return Double.isInfinite(x) || Double.isInfinite(y) || Double.isInfinite(z);
    }

    @Override
    public CoordinateSystem getSpace() {
        return coordinates;
    }

    @Override
    public boolean equals(VectorCartesian3D rhs, double tolerance) {
        return FastMath.abs(x - rhs.x) < tolerance
            && FastMath.abs(y - rhs.y) < tolerance 
            && FastMath.abs(z - rhs.z) < tolerance;
    }

    @Override
    public VectorCartesian3D cross(VectorCartesian3D rhs) {
        return new VectorCartesian3D(this.y * rhs.z - this.z * rhs.y
                                    ,this.z * rhs.x - this.x * rhs.z
                                    ,this.x * rhs.y - this.y * rhs.x);
    }

    @Override
    public int getDimension() {
        return coordinates.getDimension();
    }

    /**
     * 
     */
    @Override
    public boolean equals(Object obj)
    {
        if (obj instanceof VectorCartesian3D)
        {
            final VectorCartesian3D rhs = (VectorCartesian3D) obj;
            return (x == rhs.x) && (y == rhs.y) && (z == rhs.z);
        }
        return false;
    }

    public static final class Cartesian3D implements CoordinateSystem
    {
        private static Cartesian3D instance = new Cartesian3D();

        private Cartesian3D(){}

        public static Cartesian3D getInstance()
        {
            return instance;
        }

        @Override
        public int getDimension() {
            return 3;
        }
    }
}
