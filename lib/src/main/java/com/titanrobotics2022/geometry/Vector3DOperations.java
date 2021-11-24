package com.titanrobotics2022.geometry;

public interface Vector3DOperations<S extends CoordinateSystem> extends AbstractVectorOperations<S> {
    
    public S cross(S rhs);
}
