package com.titanrobotics2022.geometry.experimentalgeometry3;

public interface AbstractVectorOperations<S extends CoordinateSystem> extends AbstractPoint<S> {

    /** Get the null vector of the vectorial space or origin point of the affine space.
     * @return null vector of the vectorial space or origin point of the affine space
     */
    S getZero();

    S plus(S rhs);

    S minus(S rhs);

    S scalarMultiply(double scalar);

    S negate();

    double magnitude();

    double magnitudeSquared();

    S unitVector();

    double dot(S rhs);

    S projectOnto(S targetVec);
}
