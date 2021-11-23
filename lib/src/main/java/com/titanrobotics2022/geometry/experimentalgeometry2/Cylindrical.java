package com.titanrobotics2022.geometry.experimentalgeometry2;

public class Cylindrical implements Space {
    private static Cylindrical instance = new Cylindrical();

    private Cylindrical(){}

    public static Cylindrical getInstance()
    {
        return instance;
    }

    @Override
    public int getDimension() {
        return 3;
    }
}
