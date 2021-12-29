package com.titanrobotics2022.geometry;

/**
 *  Represents 2-Dimensional general operations that can be performed on all 2-Dimensional geometric vectors.
 *  @param <S> Type of the 2-Dimensional coordinate system that the vector is defined by.
 *  @implNote Does not check if the coordinate system contains 2 dimensions.
 *  @implNote All operations can be optimized for different vector spaces in implementing classes.
 */
public interface Vector2DOperations<S extends CoordinateSystem> extends AbstractVectorOperations<S> {
    
    /**
     *  The cross product of two vectors.
     *  In two dimensions such a cross product has no z dimension which means that
     *  lhs and rhs vectors are coplanar so resulting cross product only contains the z dimension.
     *  <p>result = lhs X rhs
     *  @apiNote Useful for computing surface normals found in graphics computation.
     *  @param rhs The rhs vector of the cross product.
     *  @return The z component of the cross product (can be negative).
     */
    public double cross(S rhs);
}
