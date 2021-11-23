package com.titanrobotics2022.geometry.experimentalgeometry2;

public interface Vector<S extends Space> extends Point<S> {

    /** Get the null vector of the vectorial space or origin point of the affine space.
     * @return null vector of the vectorial space or origin point of the affine space
     */
    Vector<S> getZero();

    Vector<S> plus(Vector<S> rhs);

    Vector<S> minus(Vector<S> rhs);

    Vector<S> scalarMultiply(double a);

    Vector<S> negate();

    double magnitude();

    double magnitudeSquared();

    Vector<S> unitVector();

    double dot(Vector<S> rhs);

    boolean isInfinite();

}