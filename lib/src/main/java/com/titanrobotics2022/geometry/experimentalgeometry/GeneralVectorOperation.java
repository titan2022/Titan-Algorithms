package com.titanrobotics2022.geometry.experimentalgeometry;

public interface GeneralVectorOperation<S>{
    
    public S plus(S rhs);

    public S minus(S rhs);

    public S scalarMultiply(double scalar);

    public S scalarDivide(double divisor);

    public S negate();

    public double magnitude();

    public double magnitudeSquared();

    public S unitVector();

    public double dot(S rhs);

    public S projectOnto(S otherVec);
}
