package com.titanrobotics2022.geometry.experimentalgeometry2;

public class Cartesian3D implements Space {
    private static Cartesian3D instance = new Cartesian3D();

    private Cartesian3D(){}

    public static Cartesian3D getInstance()
    {
        return instance;
    }

    @Override
    public int getDimension() {
        return 3;
    }
}