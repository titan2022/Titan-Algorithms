package com.titanrobotics2022.sensorfusion;

import org.ejml.data.DMatrixRMaj;
import org.ejml.simple.SimpleMatrix;
import static org.ejml.dense.row.CommonOps_DDRM.*;

/**
 * A modified Kalman Filter
 * 
 * This variant of a Kalman Filter supports combining estimates of multiple
 * derivatives of the target value. For instance, velocity and acceleration
 * estimates can be used to update predictions of position.
 */
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

    /**
     * Constructs a new KalmanFilter.
     * 
     * All derivates of the target quantity are assumed to have zero expectation
     * and infinite variance.
     * 
     * @param degree  The number of derivatives of the target quantity to use.
     * @param mean  The prior expectation of the target quantity.
     * @param cov  The covariance of the prior knowledge of the target quantity.
     * @param timeOffset  The time the prior knowledge applies to.
     */
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
    /**
     * Constructs a new KalmanFilter.
     * 
     * Time is assumed to be measure realative to the time of the prior knowledge
     * passed to this constructor.
     * 
     * @param degree  The number of derivatives of the target quantity to use.
     * @param mean  The prior expectation of the target quantity.
     * @param cov  The covariance of the prior knowledge of the target quantity.
     */
    public KalmanFilter(int degree, SimpleMatrix mean, SimpleMatrix cov) {
        this(degree, mean, cov, 0.0);
    }
    /**
     * Constructs a new KalmanFilter.
     * 
     * The target quantity and all derivatives are assumed to have an expectation
     * of zero and infinite variance at the given time.
     * 
     * @param degree  The number of derivatives of the target quantity to use.
     * @param timeOffset  The prior expectation of the target quantity.
     */
    public KalmanFilter(int degree, double timeOffset) {
        this(degree, new SimpleMatrix(degree, 1), new SimpleMatrix(degree, degree), timeOffset);
    }
    /**
     * Constructs a new KalmanFilter.
     * 
     * The target quantity and all derivatives are assumed to have an expectation
     * of zero mean and infinite variance at time 0.
     * 
     * @param degree  The number of derivatives of the target quantity to use.
     */
    public KalmanFilter(int degree) {
        this(degree, 0.0);
    }

    /**
     * Sets the internal state of this KalmanFilter.
     * 
     * This method overwrites any previous updates to the state of this
     * KalmanFilter at the specified degree of differentiation.
     * 
     * @param degree  The degree of the derivative of the target quantity to set the state for.
     * @param mean  The new expectation of this derivative of the target quantity.
     * @param cov  The new covariance of this derivative of the target quantity.
     * @param time  The new time of last update for this derivative of the target quantity.
     */
    public void set(int degree, DMatrixRMaj mean, DMatrixRMaj cov, double time) {
        means[degree].set(mean);
        covs[degree].set(cov);
        lastUpdated[degree] = time;
    }
    /**
     * Sets the internal state of this KalmanFilter.
     * 
     * This method overwrites any previous updates to the state of this
     * KalmanFilter at the specified degree of differentiation.
     * 
     * @param degree  The degree of the derivative of the target quantity to set the state for.
     * @param mean  The new expectation of this derivative of the target quantity.
     * @param cov  The new covariance of this derivative of the target quantity.
     * @param time  The new time of last update for this derivative of the target quantity.
     */
    public void set(int degree, SimpleMatrix mean, SimpleMatrix cov, double time) {
        set(degree, mean.getDDRM(), cov.getDDRM(), time);
    }

    /**
     * Calculates the posterior distribution of a quantity at a given time.
     * 
     * @param degree  The degree of the derivative of the target quantity to
     *  compute the distribution of.
     * @param time  The time to compute the posterior distribution for.
     */
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
    /**
     * Finds the expectation of a quantity at a given time.
     * 
     * @param degree  The degree of the derivative of the target quantity to
     *  compute the expectation of.
     * @param time  The time to compute the expectation for.
     * @return  The expected value of the given derivative of the target quantity
     *  at the specified time.
     */
    public SimpleMatrix getPred(int degree, double time) {
        calcPred(degree, time);
        return new SimpleMatrix(pred);
    }
    /**
     * Finds the expectation of the target quantity at a given time.
     * 
     * @param time  The time to compute the expectation for.
     * @return  The expected value of the target quantity at the specified time.
     */
    public SimpleMatrix getPred(double time) {
        return getPred(0, time);
    }
    /**
     * Finds the uncertainty of a quantity at a given time.
     * 
     * @param degree  The degree of the derivative of the target quantity to
     *  compute the covariance of.
     * @param time  The time to compute the covariance for.
     * @return  The covariance of the distribution of the given derivative of the
     *  target quantity at the specified time.
     */
    public SimpleMatrix getPredCov(int degree, double time) {
        calcPred(degree, time);
        return new SimpleMatrix(predCov);
    }
    /**
     * Finds the uncertainty of the target quantity at a given time.
     * 
     * @param time  The time to compute the covariance for.
     * @return  The covariance of the distribution of the target quantity at the
     *  specified time.
     */
    public SimpleMatrix getPredCov(double time) {
        return getPredCov(0, time);
    }

    public double getLastUpdated(int degree) {
        return lastUpdated[degree];
    }

    /**
     * Updates the state of this KalmanFilter.
     * 
     * @param degree  
     * @param obs
     * @param obsPrec
     * @param time
     * @return
     */
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
