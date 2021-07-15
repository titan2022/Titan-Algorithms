package com.titanrobotics2022.geometry.experimentalgeometry;

public class Vector<S extends GeneralVectorOperation<S>> implements GeneralVectorOperation<Vector<S>>{
    public final S value;

    public Vector(S headCoordinate)
    {
        this.value = headCoordinate;
    }

    @Override
    public Vector<S> plus(Vector<S> rhs) {
        return new Vector<S>(value.plus(rhs.value));
    }

    @Override
    public Vector<S> minus(Vector<S> rhs) {
        return new Vector<S>(value.minus(rhs.value));
    }

    @Override
    public Vector<S> scalarMultiply(double scalar) {
        return new Vector<S>(value.scalarMultiply(scalar));
    }

    @Override
    public Vector<S> scalarDivide(double divisor) {
        return new Vector<S>(value.scalarDivide(divisor));
    }

    @Override
    public Vector<S> negate() {
        return new Vector<S>(value.negate());
    }

    @Override
    public double magnitude() {
        return value.magnitude();
    }

    @Override
    public double magnitudeSquared() {
        return value.magnitudeSquared();
    }

    @Override
    public Vector<S> unitVector() {
        return new Vector<S>(value.unitVector());
    }

    @Override
    public double dot(Vector<S> rhs) {
        return value.dot(rhs.value);
    }

    @Override
    public Vector<S> projectOnto(Vector<S> otherVec) {
        return new Vector<S>(value.projectOnto(otherVec.value));
    }
}
