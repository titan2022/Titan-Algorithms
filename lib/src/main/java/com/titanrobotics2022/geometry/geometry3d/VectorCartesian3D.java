package com.titanrobotics2022.geometry.geometry3d;

import com.titanrobotics2022.geometry.CoordinateSystem;
import com.titanrobotics2022.geometry.Vector3DOperations;

import org.apache.commons.math3.util.FastMath;

/**
 *  A cartesian 3D geometric vector of the ordered coordinates (x,y,z).
 */
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
     *  Constructs a Cartesian 3D vector with ordered coordinates (x,y,z)
     *  @param x component (abscissa)
     *  @param y component (ordinate)
     *  @param z component (applicate)
     */
    public VectorCartesian3D(double x, double y, double z)
    {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    /**
     *  {@inheritDoc}
     *  Cartesian 3D origin is (0,0,0)
     *  @return zero vector (0,0,0)
     */
    @Override
    public VectorCartesian3D getZero() {
        return ZERO;
    }

    /**
     *  {@inheritDoc}
     */
    @Override
    public VectorCartesian3D plus(VectorCartesian3D rhs) {
        return new VectorCartesian3D(this.x + rhs.x, this.y + rhs.y, this.z + rhs.z);
    }

    /**
     *  {@inheritDoc}
     */
    @Override
    public VectorCartesian3D minus(VectorCartesian3D rhs) {
        return new VectorCartesian3D(this.x - rhs.x, this.y - rhs.y, this.z - rhs.z);
    }

    /**
     *  {@inheritDoc}
     */
    @Override
    public VectorCartesian3D scalarMultiply(double scalar) {
        return new VectorCartesian3D(this.x * scalar, this.y * scalar, this.z * scalar);
    }

    /**
     *  {@inheritDoc}
     */
    @Override
    public VectorCartesian3D negate() {
        return new VectorCartesian3D(-x, -y, -z);
    }

    /**
     *  {@inheritDoc}
     */
    @Override
    public double magnitude() {
        return FastMath.sqrt(x * x + y * y + z * z);
    }

    /**
     *  {@inheritDoc}
     */
    @Override
    public double magnitudeSquared() {
        return x * x + y * y + z * z;
    }

    /**
     *  {@inheritDoc}
     */
    @Override
    public VectorCartesian3D unitVector() {
        double mag2 = magnitudeSquared();
        if (mag2 == 0)
            return ZERO;
        else
            return scalarMultiply(1.0 / FastMath.sqrt(mag2));
    }

    /**
     *  {@inheritDoc}
     *  <p>result = sum(ai * bi) from i to 3, i = 1
     */
    @Override
    public double dot(VectorCartesian3D rhs) {
        return this.x * rhs.x + this.y * rhs.y + this.z * rhs.z;
    }

    /**
     *  {@inheritDoc}
     */
    @Override
    public VectorCartesian3D projectOnto(VectorCartesian3D targetVec) {
        VectorCartesian3D unitVec = targetVec.unitVector();
        return unitVec.scalarMultiply(dot(unitVec));
    }

    /**
     *  {@inheritDoc}
     */
    @Override
    public double azimuthalAngle() {
        double answer = FastMath.atan2(y, x);
        if (answer < 0)
            answer += 2 * Math.PI;
        return answer;
    }

    /**
     *  {@inheritDoc}
     */
    @Override
    public boolean isNaN() {
        return Double.isNaN(x) || Double.isNaN(y) || Double.isNaN(z);
    }

    /**
     *  {@inheritDoc}
     */
    @Override
    public boolean isInfinite() {
        return Double.isInfinite(x) || Double.isInfinite(y) || Double.isInfinite(z);
    }

    /**
     *  {@inheritDoc}
     *  The coordinate system is Cartesian 3D (x,y,z).
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
    public boolean equals(VectorCartesian3D rhs, double tolerance) {
        return FastMath.abs(x - rhs.x) < tolerance
            && FastMath.abs(y - rhs.y) < tolerance 
            && FastMath.abs(z - rhs.z) < tolerance;
    }

    /**
     *  {@inheritDoc}
     */
    @Override
    public VectorCartesian3D cross(VectorCartesian3D rhs) {
        return new VectorCartesian3D(this.y * rhs.z - this.z * rhs.y
                                    ,this.z * rhs.x - this.x * rhs.z
                                    ,this.x * rhs.y - this.y * rhs.x);
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
        if (obj instanceof VectorCartesian3D)
        {
            final VectorCartesian3D rhs = (VectorCartesian3D) obj;
            return (x == rhs.x) && (y == rhs.y) && (z == rhs.z);
        }
        return false;
    }

    private final static double delta = 1e-15;

    /**
     *  Converts a Cartesian 3D vector into a Cylindrical vector (r, theta (radians), z).
     *  @return The Cylindrical representation of the Cartesian 3D vector.
     *  If the Cartesian 3D vector is (0,0,0), then the returning
     *  vector will point along the Polar subvector pole (theta = 0 radians). 
     */
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

    /**
     *  Converts a Cartesian 3D vector into a Spherical vector (rho, theta (radians), phi (radians)).
     *  @return The Spherical representation of the Cartesian 3D vector.
     *  If the Cartesian 3D vector is (0,0,0), then the returning
     *  vector will point along the Polar subvector pole (theta = 0 radians). 
     */
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

    /**
     *  Computes the shortest angle between two vectors.
     *  @param v1 Vector 1.
     *  @param v2 Vector 2.
     *  @return Angle in radians.
     */
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

    /**
     *  A singleton class representing the Cartesian 3D coordinate system.
     *  Coordinates are (x,y,z)
     *  x is the abscissa
     *  y is the ordinate
     *  z is the applicate
     */
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
