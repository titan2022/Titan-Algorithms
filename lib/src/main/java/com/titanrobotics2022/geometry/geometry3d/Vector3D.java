package com.titanrobotics2022.geometry.geometry3d;

import org.apache.commons.math3.util.FastMath;

/**
 * A vector 3D class
 */
public class Vector3D {
    public final double x, y, z;

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
     * Gets the magnitude of this vector.
     * 
     * @return Magnitude of vector
     */
    public double magnitude() {
        return FastMath.sqrt(x * x + y * y + z * z);
    }

    /**
     * Gets the elevation angle of this vector.
     * 
     * @return Elevation angle of vector
     */
    public double elevationAngle() {
        return FastMath.atan2(y, x);
    }

    /**
     * Gets the azimuthal angle of this vector.
     * 
     * @return Azimuthal angle of vector
     */
    public double azimuthalAngle() {
        return FastMath.atan2(y, z);
    }

}
