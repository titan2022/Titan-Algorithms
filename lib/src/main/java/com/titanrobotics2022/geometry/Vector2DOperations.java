package com.titanrobotics2022.geometry;

public interface Vector2DOperations<S extends CoordinateSystem> extends AbstractVectorOperations<S> {
    
    public double cross(S rhs);
}
