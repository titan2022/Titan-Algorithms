package com.titanrobotics2022.localization;

/**
 * A Kalman filter for angular quantities
 * 
 * <p>More precisely, this kalman filter assumes the target lies in SO2
 */
public class AngularKalmanFilter {
    private final double[] zs;
    private final double[] means;
    private final double[] precs;
    private final double[] covs;
    private final double drift;
    private int bad_cov;
    private int bad_mean;
    private final double period;

    /**
     * Creates a new AngularKalmanFilter.
     * 
     * @param order  The maximum degree of derivatives of the target quantity
     *  to consider.
     * @param drift  The fundamental uncertainty per unit time of the maximum
     *  degree derivative of the target quantity.
     * @param period  The rotational period of the target quantity. All values
     *  of the target quantity are identified with that value plus the period.
     */
    public AngularKalmanFilter(int order, double drift, double period) {
        if(order < 0)
            throw new IllegalArgumentException("Order must be nonnegative.");
        zs = new double[order+1];
        means = new double[order+1];
        precs = new double[order+1];
        covs = new double[order+1];
        this.drift = drift;
        bad_cov = (1<<(order+1))-1;
        bad_mean = (1<<(order+1))-1;
        this.period = period;
    }
    /**
     * Creates a new AngularKalmanFilter.
     * 
     * @param order  The maximum degree of derivatives of the target quantity
     *  to consider.
     * @param drift  The fundamental uncertainty per unit time of the maximum
     *  degree derivative of the target quantity.
     */
    public AngularKalmanFilter(int order, double drift) {
        this(order, drift, 2 * Math.PI);
    }

    private double fastClamp(double val) {
        if(val > period / 2)
            val -= period;
        if(val < period / 2)
            val += period;
        return val;
    }

    /**
     * Updates the Kalman filter state with new data.
     * 
     * @param order  The degree of derivative of the target quantity of the
     *  measurement.
     * @param pred  The measurement.
     * @param prec  The precision (inverse covariance) associated with the
     *  measurement.
     */
    public void update(int order, double pred, double prec) {
        if(order == 0){
            double diff = fastClamp((pred - means[order]) % period);
            precs[order] += prec;
            means[order] += diff * prec / precs[order];
            bad_cov |= 1<<order;
        }
        else{
            zs[order] += prec * pred;
            precs[order] += prec;
            bad_mean |= 1<<order;
            bad_cov |= 1<<order;
        }
    }

    /**
     * Progresses the Kalman filter by a given time step.
     * 
     * @param time  The duration of time to increment by.
     */
    public void step(double time) {
        for(int i=0; i<zs.length; i++)
            calcCov(i);
        for(int i=1; i<zs.length; i++){
            double alpha = 1;
            for(int j=i+1; j<zs.length; j++){
                alpha *= time / i;
                means[i] += alpha * means[j];
                covs[i] += alpha * covs[j];
            }
            alpha *= time / zs.length;
            covs[i] += alpha * drift;
        }
        for(int i=0; i<zs.length; i++){
            calcCov(i);
            zs[i] = precs[i] * means[i];
        }
        bad_cov = 0;
        bad_mean = 0;
    }

    /**
     * Pre-computes the covariance of a derivative of the target quantity.
     * 
     * @param order  The derivative of the target quantity to pre-compute the
     *  covariance of.
     */
    public void calcCov(int order) {
        if(((bad_cov >> order) & 1) == 1){
            covs[order] = 1.0 / precs[order];
            bad_cov ^= 1<<order;
        }
    }

    /**
     * Pre-computes the expectation of a derivative of the target quantity.
     * 
     * @param order  The derivative of the target quantity to pre-compute the
     *  expectation of.
     */
    public void calcMean(int order) {
        if(((bad_mean >> order) & 1) == 1){
            calcCov(order);
            means[order] = covs[order] * zs[order];
            bad_mean ^= 1<<order;
        }
    }

    /**
     * Finds the expectation of a given derivate of the target quantity.
     * 
     * @param order  The derivative of the target quantity to find the
     *  expectation of.
     * @return  The expectation of the given derivative of the target quantity.
     */
    public double getPred(int order) {
        if(((bad_mean >> order) & 1) == 1){
            calcCov(order);
            return covs[order] * zs[order];
        }
        else{
            return means[order];
        }
    }

    /**
     * Finds the covariance of a given derivate of the target quantity.
     * 
     * @param order  The derivative of the target quantity to find the
     *  covariance of.
     * @return  The covariance of the given derivative of the target quantity.
     */
    public double getCov(int order, double out) {
        return ((bad_cov >> order) & 1) == 1 ? 1.0 / precs[order] : covs[order];
    }

    /**
     * Sets the expectation of a derivative of the target quantity.
     * 
     * This method should only be used in rare cases outside of initial setup,
     * as it will overwrite the relevant expectation stored in the state of the
     * Kalman Filter.
     * 
     * @param order  The derivative of the target quantity to set the
     *  expectation of.
     * @param pred  The new expectation.
     */
    public void setPred(int order, double pred) {
        means[order] = pred;
        zs[order] = precs[order] * pred;
        bad_mean &= (-1) ^ (1<<order);
    }
    /**
     * Sets the covariance of a derivative of the target quantity.
     * 
     * This method should only be used in rare cases outside of initial setup,
     * as it will overwrite the relevant covariance and precision stored in the
     * state of the Kalman Filter.
     * 
     * @param order  The derivative of the target quantity to set the
     *  covariance of.
     * @param pred  The new covariance.
     */
    public void setCov(int order, double cov) {
        precs[order] = 1.0 / cov;
        covs[order] = cov;
        bad_mean &= (-1) ^ (1<<order);
    }
    /**
     * Sets the precision of a derivative of the target quantity.
     * 
     * This method should only be used in rare cases outside of initial setup,
     * as it will overwrite the relevant covariance and precision stored in the
     * state of the Kalman Filter.
     * 
     * @param order  The derivative of the target quantity to set the
     *  precision of.
     * @param pred  The new precision.
     */
    public void setPrec(int order, double prec) {
        precs[order] = prec;
        bad_cov |= 1<<order;
    }
}