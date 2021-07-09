package com.titanrobotics2022.geometry.geometry3d;

import org.apache.commons.math3.util.FastMath;

/**
 * A vector 3D class
 */
public class Vector3D {
    /** Origin (coordinates: 0, 0). */
    public static final Vector3D ZERO = new Vector3D(0, 0, 0);

    /** A vector with all coordinates set to NaN. */
    public static final Vector3D NAN = new Vector3D(Double.NaN, Double.NaN, Double.NaN);

    /** A vector with all coordinates set to positive infinity. */
    public static final Vector3D POSITIVE_INFINITY = new Vector3D(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);

    /** A vector with all coordinates set to negative infinity. */
    public static final Vector3D NEGATIVE_INFINITY = new Vector3D(Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY);
    
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
    public Vector3D(double x, double y, double z)
    {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    /**
     * Adds two vectors together
     * @param rhs the vector to be added
     * @return the resulting vector
     */
    public Vector3D plus(Vector3D rhs)
    {
        return new Vector3D(this.x + rhs.x, this.y + rhs.y, this.z + rhs.z);
    }

    /**
     * Subtracts the explicit vector from the implicit vector
     * @param rhs the vector to be subtracted
     * @return the resulting vector
     */
    public Vector3D minus(Vector3D rhs)
    {
        return new Vector3D(this.x - rhs.x, this.y - rhs.y, this.z - rhs.z);
    }

    /**
     * 
     * @param scalar
     * @return
     */
    public Vector3D scalarMultiply(double scalar)
    {
        return new Vector3D(this.x * scalar, this.y * scalar, this.z * scalar);
    }

    /**
     * 
     * @param scalar
     * @return
     */
    public Vector3D scalarDivide(double divisor)
    {
        return new Vector3D(this.x / divisor, this.y / divisor, this.z / divisor);
    }

    /**
     * 
     * @return
     */
    public Vector3D negate()
    {
        return new Vector3D(-x, -y, -z);
    }

    /**
     * 
     * @return
     */
    public double magnitude()
    {
        return FastMath.sqrt(x * x + y * y + z * z);
    }

    /**
     * 
     * @return
     */
    public double magnitudeSquared()
    {
        return x * x + y * y + z * z;
    }

    /**
     * 
     * @return
     */
    public Vector3D unitVector()
    {
        double mag2 = magnitudeSquared();
        if (mag2 == 0)
            return ZERO;
        else
            return scalarDivide(FastMath.sqrt(mag2));
    }

    /**
     * 
     * @param rhs
     * @return
     */
    public double dot(Vector3D rhs)
    {
        return this.x * rhs.x + this.y * rhs.y + this.z * rhs.z;
    }

    /**
     * 
     * @param rhs
     * @return
     */
    public Vector3D cross(Vector3D rhs)
    {
        return new Vector3D(this.y * rhs.z - this.z * rhs.y
                           ,this.z * rhs.x - this.x * rhs.z
                           ,this.x * rhs.y - this.y * rhs.x);
    }

    /**
     * 
     * @param otherVec
     * @return
     */
    public Vector3D projectOnto(Vector3D otherVec)
    {
        Vector3D unitVec = otherVec.unitVector();
        return unitVec.scalarMultiply(dot(unitVec));
    }

    /**
     * 
     * @param v1
     * @param v2
     * @return
     */
    public static double shortestAngleBetween(Vector3D v1, Vector3D v2)
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
     * 
     */
    @Override
    public boolean equals(Object obj)
    {
        if (obj instanceof Vector3D)
        {
            final Vector3D rhs = (Vector3D) obj;
            return (x == rhs.x) && (y == rhs.y) && (z == rhs.z);
        }
        return false;
    }
}
