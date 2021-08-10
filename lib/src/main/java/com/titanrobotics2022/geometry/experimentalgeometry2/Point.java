package com.titanrobotics2022.geometry.experimentalgeometry2;

public interface Point<S extends Space> {

    /**
     * Returns true if any of the coordinates are NaN.
     * @return true if any of the coordinates are NaN
     */
    boolean isNaN();

    // TODO: Decide if to add
    /** Get the space to which the point belongs.
     * @return containing space
     */
    //Space getSpace();
}
