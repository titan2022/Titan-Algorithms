package com.titanrobotics2022.geometry;

/**
 * Represents a generic geometric point.
 * @param <S> Type of the coordinate system that the point is defined by.
 */
public interface AbstractPoint<S extends CoordinateSystem> {

    /**
     *  Checks if any of the coordinates are NaN; false otherwise.
     *  @return True if any of the coordinates are NaN; false otherwise.
     */
    boolean isNaN();

    /**
     *  Checks if any of the coordiantes are positive or negative infinity; false otherwise.
     *  @return True if any of the coordiantes are positive or negative infinity; false otherwise.
     */
    boolean isInfinite();

    /**
     *  Get the space to which the point belongs.
     *  @return Returns the defining space type.
     */
    CoordinateSystem getSpace();

    /**
     *  Asserts that two coordinates are equal with tolerance.
     *  @param rhs Coordinate to be compared to implicit coordinate.
     *  @param tolerance Maximum tolerance difference between coordinates.
     *  @return True if coordinates are within tolerance of each other; false otherwise.
     */
    boolean equals(S rhs, double tolerance);
}
