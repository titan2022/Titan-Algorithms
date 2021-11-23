package com.titanrobotics2022.geometry.experimentalgeometry2;

public interface Vector3D<S extends Space> extends Vector<S> {
    
    public S cross(S rhs);
}
