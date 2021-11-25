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
