package com.titanrobotics2022.sensorfusion;

import org.ejml.data.DMatrixRMaj;
import org.ejml.simple.SimpleMatrix;
import static org.ejml.dense.row.CommonOps_DDRM.*;

public class KalmanFilter {
    protected final DMatrixRMaj[] means;
    protected final DMatrixRMaj[] covs;
    protected final double[] lastUpdated;
    private final DMatrixRMaj pred;
    private final DMatrixRMaj predCov;
    private final DMatrixRMaj prec;
    private final DMatrixRMaj z;
    private double predTime = 0;
    private int predDegree = -1;

    public KalmanFilter(int degree, SimpleMatrix mean, SimpleMatrix cov, double timeOffset) {
        if(degree < 0)
            throw new IllegalArgumentException("Cannot construct a Kalman Filter with a negative degree.");
        if(mean.numCols() != 1)
            throw new IllegalArgumentException("mean vector is not a column vector.");
        if(cov.numRows() != mean.numRows() || cov.numCols() != mean.numRows())
            throw new IllegalArgumentException("Covariance matrix should be square with the same dimension as the mean vector.");
        if(cov.determinant() < 0 || !cov.isIdentical(cov.transpose(), 0))
            throw new IllegalArgumentException("Covariance matrix must be symmetric positive semi-definite.");
        means = new DMatrixRMaj[degree+1];
        covs = new DMatrixRMaj[degree+1];
        lastUpdated = new double[degree+1];
        for(int i=0; i<degree+1; i++){
            means[i] = new DMatrixRMaj(mean.numRows(), mean.numCols());
            covs[i] = new DMatrixRMaj(cov.getDDRM());
            lastUpdated[i] = timeOffset;
        }
        pred = means[0].createLike();
        predCov = covs[0].createLike();
        prec = covs[0].createLike();
        z = means[0].createLike();
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

    public void set(int degree, DMatrixRMaj mean, DMatrixRMaj cov, double time) {
        means[degree].set(mean);
        covs[degree].set(cov);
        lastUpdated[degree] = time;
    }
    public void set(int degree, SimpleMatrix mean, SimpleMatrix cov, double time) {
        set(degree, mean.getDDRM(), cov.getDDRM(), time);
    }

    protected void calcPred(int degree, double time) {
        if(predTime == time && predDegree == degree) return;
        add(means[degree], 0, pred);
        add(covs[degree], 0, predCov);
        double coef;
        double fact = 1;
        for(int i=degree+1; i<means.length; i++){
            fact *= i;
            coef = Math.pow(time - lastUpdated[i], i-degree) / fact;
            addEquals(pred, coef, means[i]);
            addEquals(predCov, coef, covs[i]);
        }
        predTime = time;
        predDegree = degree;
    }
    public SimpleMatrix getPred(int degree, double time) {
        calcPred(degree, time);
        return new SimpleMatrix(pred);
    }
    public SimpleMatrix getPred(double time) {
        return getPred(0, time);
    }
    public SimpleMatrix getPredCov(int degree, double time) {
        calcPred(degree, time);
        return new SimpleMatrix(predCov);
    }
    public SimpleMatrix getPredCov(double time) {
        return getPredCov(0, time);
    }

    public double getLastUpdated(int degree) {
        return lastUpdated[degree];
    }

    public boolean update(int degree, DMatrixRMaj obs, DMatrixRMaj obsPrec, double time) {
        if(time < lastUpdated[degree])
            return false;
        calcPred(degree, time);
        if(!invertSPD(predCov, prec))
            pinv(predCov, prec);
        add(prec, obsPrec, predCov);
        if(!invertSPD(predCov, covs[degree]))
            pinv(predCov, covs[degree]);
        mult(prec, pred, z);
        multAdd(z, obsPrec, obs);
        mult(covs[degree], z, means[degree]);
        predDegree = -1;
        return true;
    }
    public boolean update(int degree, SimpleMatrix obs, SimpleMatrix obsPrec, double time) {
        return update(degree, obs.getDDRM(), obsPrec.getDDRM(), time);
    }
}
