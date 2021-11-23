package com.titanrobotics2022.geometry.experimentalgeometry2;

public interface Point<S extends Space> {

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
    Vector<S> minus(Point<S> rhs);
    
    /**
     * Computes a new point using a displacement vector for the implicit point.
     * @param rhs Displacement vector.
     * @return A point displaced from the original point.
     */
    Point<S> plus(Vector<S> rhs);

    /** Get the space to which the point belongs.
     * @return containing space
     */
    Space getSpace();
}
