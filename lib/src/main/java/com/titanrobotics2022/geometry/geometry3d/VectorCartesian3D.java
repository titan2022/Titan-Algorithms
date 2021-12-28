package com.titanrobotics2022.geometry.geometry3d;

import com.titanrobotics2022.geometry.CoordinateSystem;
import com.titanrobotics2022.geometry.Vector3DOperations;

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

    private final static double delta = 1e-15;

    public VectorCylindrical toCylindrical()
    {
        // Same implementation as polar but with added z component
        double rsquaredForTheta = x * x + y * y;
        if(rsquaredForTheta == 0)
            return VectorCylindrical.ZERO;
        else
        {
            double theta = FastMath.atan2(y, x);
            if(theta < -delta)
                theta += 2 * Math.PI;
            else if(theta < delta) // Sets theta to be 0 exactly, theta was previous evaluated to be greater than negative delta
                theta = 0;
            return new VectorCylindrical(FastMath.sqrt(rsquaredForTheta), theta, z);
        }   
    }

    public VectorSpherical toSpherical()
    {
        double rhosquared = magnitudeSquared();
        if(rhosquared == 0)
            return VectorSpherical.ZERO;
        else
        {
            double theta;
            if(x * x + y * y < delta)
                theta = 0;
            else
            {
                theta = FastMath.atan2(y, x);
                if(theta < -delta)
                    theta += 2 * Math.PI;
                else if(theta < delta) // Sets theta to be 0 exactly, theta was previous evaluated to be greater than negative delta
                    theta = 0;
            }

            double rhoMag = FastMath.sqrt(rhosquared);
            double phi = FastMath.acos(z / rhoMag);
            return new VectorSpherical(rhoMag, theta, phi);
        }
    }

    public static double shortestAngleBetween(VectorCartesian3D v1, VectorCartesian3D v2)
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

    public static final class Cartesian3D implements CoordinateSystem
    {
        private final static Cartesian3D instance = new Cartesian3D();

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
