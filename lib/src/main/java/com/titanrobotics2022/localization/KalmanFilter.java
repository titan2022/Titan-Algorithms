package com.titanrobotics2022.localization;

import org.ejml.data.DMatrix3;
import org.ejml.data.DMatrix3x3;

import static org.ejml.dense.fixed.CommonOps_DDF3.*;

/**
 * A Kalman Filter with higher-order derivative information.
 */
public class KalmanFilter {
    //[x, y, phi]
    private final DMatrix3[] zs;
    private final DMatrix3x3[] precs;
    private final DMatrix3[] means;
    private final DMatrix3x3[] covs;
    private final DMatrix3x3 drift;
    private final DMatrix3 v = new DMatrix3();
    private final DMatrix3x3 m = new DMatrix3x3();
    private int bad_cov;
    private int bad_mean;

    /**
     * Creates a new KalmanFilter.
     * 
     * @param order  The maximum degree of derivatives of the target quantity
     *  to consider.
     * @param drift  The fundamental uncertainty per unit time of the maximum
     *  degree derivative of the target quantity.
     */
    public KalmanFilter(int order, DMatrix3x3 drift) {
        if(order < 0)
            throw new IllegalArgumentException("Order must be nonnegative.");
        this.drift = drift.copy();
        zs = new DMatrix3[order+1];
        precs = new DMatrix3x3[order+1];
        means = new DMatrix3[order+1];
        covs = new DMatrix3x3[order+1];
        for(int i=0; i<=order; i++){
            zs[i] = new DMatrix3();
            precs[i] = new DMatrix3x3();
            means[i] = new DMatrix3();
            covs[i] = new DMatrix3x3();
        }
        bad_cov = (1<<(order+1))-1;
        bad_mean = (1<<(order+1))-1;
    }

    /**
     * Updates the Kalman Filter state with new data.
     * 
     * @param order  The degree of derivative of the target quantity of the
     *  measurement.
     * @param pred  The measurement.
     * @param prec  The precision (inverse covariance) associated with the
     *  measurement.
     */
    public void update(int order, DMatrix3 pred, DMatrix3x3 prec) {
        mult(prec, pred, v);
        addEquals(zs[order], v);
        addEquals(precs[order], prec);
        bad_cov |= 1<<order;
        bad_mean |= 1<<order;
    }

    /**
     * Progresses the Kalman Filter by a given time step.
     * 
     * @param time  The duration of time to increment by.
     */
    public void step(double time) {
        double alpha;
        for(int i=0; i<zs.length; i++){
            safeInvert(precs[i], covs[i]);
            mult(covs[i], zs[i], means[i]);
        }
        for(int i=1; i<zs.length; i++){
            alpha = 1;
            for(int j=i+1; j<zs.length; j++){
                alpha *= time / i;
                scale(alpha, means[j], v);
                scale(alpha, covs[j], m);
                addEquals(means[i], means[j]);
                addEquals(covs[i], covs[j]);
            }
            alpha *= time / zs.length;
            scale(alpha, drift, m);
            addEquals(covs[i], m);
        }
        for(int i=0; i<zs.length; i++){
            safeInvert(covs[i], precs[i]);
            mult(precs[i], means[i], zs[i]);
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
            safeInvert(precs[order], covs[order]);
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
            mult(covs[order], zs[order], means[order]);
            bad_mean ^= 1;
        }
    }

    /**
     * Finds the expectation of a given derivative of the target quantity.
     * 
     * @param order  The derivative of the target quantity to find the
     *  expectation of.
     * @param out  A vector to populate with the expectation.
     */
    public void getPred(int order, DMatrix3 out) {
        if(((bad_mean >> order) & 1) == 1){
            calcCov(order);
            mult(covs[order], zs[order], out);
        }
        else{
            out.set(means[order]);
        }
    }
    /**
     * Finds the expectation of a given derivate of the target quantity.
     * 
     * @param order  The derivative of the target quantity to find the
     *  expectation of.
     * @return  The expectation of the given derivative of the target quantity,
     *  as a vector.
     */
    public DMatrix3 getPred(int order) {
        DMatrix3 res = new DMatrix3();
        getPred(order, res);
        return res;
    }

    /**
     * Finds the covariance of a given derivate of the target quantity.
     * 
     * @param order  The derivative of the target quantity to find the
     *  covariance of.
     * @param out  A matrix to populate with the covariance.
     */
    public void getCov(int order, DMatrix3x3 out) {
        if(((bad_cov >> order) & 1) == 1)
            safeInvert(precs[order], out);
        else
            out.set(covs[order]);
    }
    /**
     * Finds the covariance of a given derivate of the target quantity.
     * 
     * @param order  The derivative of the target quantity to find the
     *  covariance of.
     * @return  The covariance of the given derivative of the target quantity,
     *  as a matrix.
     */
    public DMatrix3x3 getCov(int order) {
        DMatrix3x3 res = new DMatrix3x3();
        getCov(order, res);
        return res;
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
    public void setPred(int order, DMatrix3 pred) {
        mult(precs[order], pred, zs[order]);
        bad_mean |= 1<<order;
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
     * @param pred  The new covariance matrix.
     */
    public void setCov(int order, DMatrix3x3 cov) {
        safeInvert(cov, precs[order]);
        covs[order].set(cov);
        bad_cov ^= (bad_cov >> order) & 1;
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
     * @param pred  The new precision matrix.
     */
    public void setPrec(int order, DMatrix3x3 prec) {
        precs[order].set(prec);
        bad_cov |= 1<<order;
    }

    /**
     * Safely computes the inverse of a 2x2 symmetric PSD matrix.
     * 
     * This method falls back on the pseudoinverse if the inverse cannot be
     * computed.
     * 
     * @param a  The matrix to invert.
     * @param inv  The inverted matrix. Can be the same as `a`.
     */
    private static void safeInvert(DMatrix3x3 a, DMatrix3x3 inv) {
        if(!invert(a, inv)){
            double sigma = 1 / (a.a11 + a.a22);
            scale(sigma * sigma, a, inv);
        }
    }
}
