package com.titanrobotics2022.geometry;

/**
 *  Represents general operations that can be performed on all geometric vectors.
 *  @param <S> Type of the coordinate system that the vector is defined by.
 *  @implNote All operations can be optimized for different vector spaces in implementing classes.
 */
public interface AbstractVectorOperations<S extends CoordinateSystem> extends AbstractPoint<S> {

    /** 
     *  Get the null vector of the vector space or origin point of the affine space.
     *  @return null vector of the vector space or origin point of the affine space
     */
    S getZero();

    /**
     *  Perform addition on the lhs implicit vector with the rhs explicit vector.
     *  <p>result = lhs + rhs
     *  @param rhs The vector addend.
     *  @return Sum of two vectors.
     */
    S plus(S rhs);

    /**
     *  Perform subtraction on the lhs implicit vector with the rhs explicit vector.
     *  <p>result = lhs - rhs
     *  @param rhs The vector subtractend.
     *  @return Difference of two vectors.
     */
    S minus(S rhs);

    /**
     *  Perform scalar multiplication to the implicit vector.
     *  <p>result = scalar * vector
     *  @apiNote This method should be used to divide by scaling the vector by 1 / divisor.
     *  User will need to check for zero division.
     *  @param scalar Coefficient to scale vector by.
     *  @return The scaled vector.
     */
    S scalarMultiply(double scalar);

    /**
     *  Same as scalar multiplication by -1 to the implicit vector.
     *  <p>result = -1 * vector
     *  @return A vector pointing in the opposite geometric direction with same magnitude.
     */
    S negate();

    /**
     *  The geometric/euclidean norm of the vector.
     *  @return The geometric length of the vector.
     */
    double magnitude();

    /**
     *  The geometric/euclidean norm of the vector squared.
     *  @implNote Should be implemented without squaring the magnitude to optimize execution.
     *  <p>Ex: Cartesian2D magnitude squared = x * x + y * y.
     *  @return The geometric length of the vector squared.
     */
    double magnitudeSquared();

    /**
     *  Computes a normalized geometric vector (magnitude 1).
     *  @return Normalizes a vector.
     */
    S unitVector();

    /**
     *  Computes the dot/scalar product of an implicit and explicit geometric vector.
     *  <p>result = lhs * rhs
     *  @param rhs A second vector.
     *  @return The dot product of lhs implicit and rhs explicit.
     */
    double dot(S rhs);

    /**
     *  The projection of implicit vector onto explicit vector.
     *  <p>result = proj(implicit) onto explicit
     *  @param targetVec Vector to project onto.
     *  @return The resulting projection.
     */
    S projectOnto(S targetVec);

    /**
     *  The signed angle made between the vector and the positive x-axis
     *  @return Angle in radians
     */
    double azimuthalAngle();
}
