package com.titanrobotics2022.geometry.experimentalgeometry2;

public interface Vector2D<S extends Space> extends Vector<S> {
    
    public double cross(S rhs);
}
