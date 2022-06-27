package com.titanrobotics2022.motion.control;

import org.ejml.simple.SimpleMatrix;

import com.titanrobotics2022.optimization.cost.QuadCost;

public class RiccatiUtils {
    /**
     * Discrete time linear system ricatti equation solved via dynamic programming (DP)
     * Input Matrices of N observations and M controls.
     * Pt−1 := Q + ATPtA − ATPtB((R + BTPtB)^−1)BTPtA
     * 
     * @see https://stanford.edu/class/ee363/lectures/allslides.pdf
     * 
     * @param A System matrix N by N.
     * @param B Input/Control matrix N by M.
     * @param Q State cost matrix N by N.
     * @param R Control cost matrix M by M.
     * @param Pt Previous or starting ricatti equation solution N by N.
     * Starting Pt is the terminal cost matrix for finite horizon.
     * Starting Pt is the state cost matrix for infinite horizon.
     * @return Pt-1
     */
    static SimpleMatrix dynamicDiscreteTimeRicattiEquation(SimpleMatrix A, SimpleMatrix B
                                                          ,QuadCost Q, QuadCost R
                                                          ,SimpleMatrix Pt)
    {
        SimpleMatrix ATPt = A.transpose().mult(Pt);
        SimpleMatrix BTPt = B.transpose().mult(Pt);
        SimpleMatrix QMatrix = Q.getCostMatrix();
        SimpleMatrix ATPtA = ATPt.mult(A);
        SimpleMatrix ATPtB = ATPt.mult(B);
        SimpleMatrix RMatrix = R.getCostMatrix();
        SimpleMatrix BTPtB = BTPt.mult(B);
        SimpleMatrix BTPtA = BTPt.mult(A);
        return QMatrix.plus(ATPtA).minus(ATPtB.mult(RMatrix.plus(BTPtB).pseudoInverse()).mult(BTPtA));
    }

    // static SimpleMatrix dynamicContinousTimeRicattiEquation(SimpleMatrix A, SimpleMatrix B
    //                                                        ,QuadCost Q, QuadCost R
    //                                                        ,SimpleMatrix Pk)
    // {
    //     
    // }
}
