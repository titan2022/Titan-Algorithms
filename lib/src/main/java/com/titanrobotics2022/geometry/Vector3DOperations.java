package com.titanrobotics2022.geometry;

/**
 *  Represents 3-Dimensional general operations that can be performed on all 3-Dimensional geometric vectors.
 *  @param <S> Type of the 3-Dimensional coordinate system that the vector is defined by.
 *  @implNote Does not check if the coordinate system contains 3 dimensions.
 *  @implNote All operations can be optimized for different vector spaces in implementing classes.
 */
public interface Vector3DOperations<S extends CoordinateSystem> extends AbstractVectorOperations<S> {
    
    /**
     *  The cross product of two vectors.
     *  <p>result = lhs X rhs
     *  @param rhs The rhs vector of the cross product.
     *  @return A 3-Dimensional vector.
     */
    public S cross(S rhs);
}
