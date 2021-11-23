package com.titanrobotics2022.geometry.experimentalgeometry2;

public class Polar implements Space {
    private static Polar instance = new Polar();

    private Polar(){}

    public static Polar getInstance()
    {
        return instance;
    }

    @Override
    public int getDimension() {
        return 2;
    }
}
