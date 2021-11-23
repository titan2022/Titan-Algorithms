package com.titanrobotics2022.geometry.experimentalgeometry3;

public interface Vector2DOperations<S extends CoordinateSystem> extends AbstractVectorOperations<S> {
    
    public double cross(S rhs);
}
