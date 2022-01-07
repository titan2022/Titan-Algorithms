package com.titanrobotics2022.sensorfusion;

import org.ejml.simple.SimpleMatrix;

public class KalmanFilter {
    protected final SimpleMatrix[] means;
    protected final SimpleMatrix[] covs;
    protected final double[] lastUpdated;

    public KalmanFilter(int degree, SimpleMatrix mean, SimpleMatrix cov, double timeOffset) {
        if(degree < 0)
            throw new IllegalArgumentException("Cannot construct a Kalman Filter with a negative degree.");
        if(mean.numCols() != 1)
            throw new IllegalArgumentException("mean vector is not a column vector.");
        if(cov.numRows() != mean.numRows() || cov.numCols() != mean.numRows())
            throw new IllegalArgumentException("Covariance matrix should be square with the same dimension as the mean vector.");
        if(cov.determinant() < 0 || !cov.isIdentical(cov.transpose(), 0))
            throw new IllegalArgumentException("Covariance matrix must be symmetric positive semi-definite.");
        means = new SimpleMatrix[degree+1];
        covs = new SimpleMatrix[degree+1];
        lastUpdated = new double[degree+1];
        for(int i=0; i<degree+1; i++){
            means[i] = new SimpleMatrix(mean.numRows(), mean.numCols());
            covs[i] = new SimpleMatrix(cov);
            lastUpdated[i] = timeOffset;
        }
    }
    public KalmanFilter(int degree, SimpleMatrix mean, SimpleMatrix cov) {
        this(degree, mean, cov, 0.0);
    }
    public KalmanFilter(int degree, double timeOffset) {
        this(degree, new SimpleMatrix(degree, 1), new SimpleMatrix(degree, degree), timeOffset);
    }
    public KalmanFilter(int degree) {
        this(degree, 0.0);
    }
}
