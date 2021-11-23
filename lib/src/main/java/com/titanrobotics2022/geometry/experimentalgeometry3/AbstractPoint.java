package com.titanrobotics2022.geometry.experimentalgeometry3;

public interface AbstractPoint<S extends CoordinateSystem> {

    /**
     * Returns true if any of the coordinates are NaN.
     * @return true if any of the coordinates are NaN
     */
    boolean isNaN();

    /**
     * Returns true if any of the coordiantes are positive or negative infinity
     * @return true if any of the coordiantes are positive or negative infinity
     */
    boolean isInfinite();

    /**
     * Computes the displacement of two points as a vector where the
     * return vector is also the vector from rhs to lhs.
     * @param rhs Right hand side point of pt - pt = vector.
     * @return Vector displacement of rhs to lhs.
     */
    S minus(S rhs);
    
    /**
     * Computes a new point using a displacement vector for the implicit point.
     * @param rhs Displacement vector.
     * @return A point displaced from the original point.
     */
    S plus(S rhs);

    /** Get the space to which the point belongs.
     * @return containing space
     */
    CoordinateSystem getSpace();
}
