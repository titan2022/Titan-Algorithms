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

    public void set(int degree, SimpleMatrix mean, SimpleMatrix cov, double time) {
        means[degree].set(mean);
        covs[degree].set(cov);
        lastUpdated[degree] = time;
    }

    public SimpleMatrix[] getFullPred(int degree, double time) {
        SimpleMatrix mean = means[degree].copy();
        SimpleMatrix cov = covs[degree].copy();
        double coef;
        double fact = 1;
        for(int i=degree+1; i<means.length; i++){
            fact *= i;
            coef = Math.pow(time - lastUpdated[i], i-degree) / fact;
            mean = mean.plus(means[i].scale(coef));
            cov = cov.plus(covs[i].scale(coef));
        }
        return new SimpleMatrix[]{mean, cov};
    }
    public SimpleMatrix[] getFullPred(double time) {
        return getFullPred(0, time);
    }
    public SimpleMatrix getPred(int degree, double time) {
        SimpleMatrix pred = means[degree].copy();
        double coef;
        double fact = 1;
        for(int i=degree+1; i<means.length; i++){
            fact *= i;
            coef = Math.pow(time - lastUpdated[i], i-degree) / fact;
            pred = pred.plus(means[i].scale(coef));
        }
        return pred;
    }
    public SimpleMatrix getPred(double time) {
        return getPred(0, time);
    }
    public SimpleMatrix getPredCov(int degree, double time) {
        SimpleMatrix predCov = covs[degree].copy();
        double coef;
        double fact = 1;
        for(int i=degree+1; i<covs.length; i++){
            fact *= i;
            coef = Math.pow(time - lastUpdated[i], i-degree) / fact;
            predCov = predCov.plus(covs[i].scale(coef));
        }
        return predCov;
    }
    public SimpleMatrix getPredCov(double time) {
        return getPredCov(0, time);
    }

    public double getLastUpdated(int degree) {
        return lastUpdated[degree];
    }

    public SimpleMatrix update(int degree, SimpleMatrix obs, SimpleMatrix prec, double time) {
        if(time < lastUpdated[degree])
            return null;
        SimpleMatrix[] prior = getFullPred(degree, time);
        SimpleMatrix postZ = prior[1].solve(prior[0]).plus(prec.mult(obs));
        SimpleMatrix postCov = (prior[1].invert().plus(prec)).invert();
        means[degree] = postCov.mult(postZ);
        covs[degree] = postCov;
        lastUpdated[degree] = time;
        return means[degree];
    }
}
