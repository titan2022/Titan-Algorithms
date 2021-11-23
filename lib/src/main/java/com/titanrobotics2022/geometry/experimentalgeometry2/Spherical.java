package com.titanrobotics2022.geometry.experimentalgeometry2;

public class Spherical implements Space {
    private static Spherical instance = new Spherical();

    private Spherical(){}

    public static Spherical getInstance()
    {
        return instance;
    }

    @Override
    public int getDimension() {
        return 3;
    }
}