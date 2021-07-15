package com.titanrobotics2022.geometry.experimentalgeometry;

public interface CoordinateTransformation<S> {
    
    public S convertTo(Class<S> coordinateSystem);
}
