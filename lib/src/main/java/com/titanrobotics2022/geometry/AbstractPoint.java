package com.titanrobotics2022.geometry;

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

    // TODO: Decide if a minus and plus operation is needed for when designing point specific classes
    /**
     * Computes the displacement of two points as a vector where the
     * return vector is also the vector from rhs to lhs.
     * @param rhs Right hand side point of pt - pt = vector.
     * @return Vector displacement of rhs to lhs.
     */
    //S minus(AbstractVectorOperations<S> rhs);
    
    /**
     * Computes a new point using a displacement vector for the implicit point.
     * @param rhs Displacement vector.
     * @return A point displaced from the original point.
     */
    //S plus(AbstractVectorOperations<S> rhs);

    /** Get the space to which the point belongs.
     * @return containing space
     */
    CoordinateSystem getSpace();

    /**
     * Assert that two coordinates are equal with tolerance.
     * @param rhs Coordinate to be compared to
     * @param tolerance maximum difference between coordinates
     * @return true if coordinates are within tolerance of each other
     */
    boolean equals(S rhs, double tolerance);
}