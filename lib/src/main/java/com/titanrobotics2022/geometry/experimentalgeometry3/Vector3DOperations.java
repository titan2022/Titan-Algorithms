package com.titanrobotics2022.geometry.experimentalgeometry3;

public interface Vector3DOperations<S extends CoordinateSystem> extends AbstractVectorOperations<S> {
    
    public S cross(S rhs);
}
