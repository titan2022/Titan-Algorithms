package com.titanrobotics2022.geometry.experimentalgeometry2;

public class Cartesian2D implements Space {
    private static Cartesian2D instance = new Cartesian2D();

    private Cartesian2D(){}

    public static Cartesian2D getInstance()
    {
        return instance;
    }

    @Override
    public int getDimension() {
        return 2;
    }
}
