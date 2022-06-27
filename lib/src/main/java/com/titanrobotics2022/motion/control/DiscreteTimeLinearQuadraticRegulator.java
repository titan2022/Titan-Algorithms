package com.titanrobotics2022.motion.control;

import org.ejml.simple.SimpleMatrix;

import com.titanrobotics2022.optimization.cost.QuadCost;

public class DiscreteTimeLinearQuadraticRegulator {
    private SimpleMatrix A, B, K, goal;
    private QuadCost Q, R, F;
    private int horizon;

    public DiscreteTimeLinearQuadraticRegulator(SimpleMatrix A, SimpleMatrix B
                                               ,QuadCost Q, QuadCost R, QuadCost F
                                               ,SimpleMatrix goal, int horizon)
    {
        this.A = A;
        this.B = B;
        this.goal = goal;
        this.Q = Q;
        this.R = R;
        this.F = F;
        this.horizon = horizon;
        SimpleMatrix Pn = F.getCostMatrix();
        SimpleMatrix Pnminus1 = RiccatiUtils.dynamicDiscreteTimeRicattiEquation(A, B, Q, R, Pn);
        for(int n = horizon; n > 1; n--)
        {
            Pn = Pnminus1;
            Pnminus1 = RiccatiUtils.dynamicDiscreteTimeRicattiEquation(A, B, Q, R, Pn);
        }
        // Kt := −((R+BTPtB)^−1)BTPtA
        SimpleMatrix BTPt = B.transpose().mult(Pnminus1);
        SimpleMatrix BTPtA = BTPt.mult(A);
        SimpleMatrix BTPtB = BTPt.mult(B);
        SimpleMatrix RMatrix = R.getCostMatrix();
        this.K = RMatrix.plus(BTPtB).pseudoInverse().negative().mult(BTPtA);
    }

    public DiscreteTimeLinearQuadraticRegulator(SimpleMatrix A, SimpleMatrix B
                                               ,QuadCost Q, QuadCost R, QuadCost F
                                               ,SimpleMatrix goal, double threshold, int maxIterations)
    {
        this.A = A;
        this.B = B;
        this.goal = goal;
        this.Q = Q;
        this.R = R;
        this.F = F;
        this.horizon = Integer.MAX_VALUE; // Infinite horizon
        SimpleMatrix Pn = Q.getCostMatrix();
        SimpleMatrix Pnminus1 = RiccatiUtils.dynamicDiscreteTimeRicattiEquation(A, B, Q, R, Pn);
        double Pdiff = Pn.minus(Pnminus1).elementMaxAbs();
        int iteration = 0;
        while(Pdiff > threshold && iteration < maxIterations)
        {
            Pn = Pnminus1;
            Pnminus1 = RiccatiUtils.dynamicDiscreteTimeRicattiEquation(A, B, Q, R, Pn);
            Pdiff = Pn.minus(Pnminus1).elementMaxAbs();
            iteration++;
        }
        if(iteration == maxIterations)
        {
            System.out.println("Warning: Discrete time LQR infinite horizon failed to converge");
        }

        // Kt := −(R+BTPtB)−1BTPtA
        SimpleMatrix BTPt = B.transpose().mult(Pnminus1);
        SimpleMatrix BTPtA = BTPt.mult(A);
        SimpleMatrix BTPtB = BTPt.mult(B);
        SimpleMatrix RMatrix = R.getCostMatrix();
        this.K = RMatrix.plus(BTPtB).pseudoInverse().negative().mult(BTPtA);
    }

    public DiscreteTimeLinearQuadraticRegulator(SimpleMatrix A, SimpleMatrix B
                                               ,QuadCost Q, QuadCost R, QuadCost F
                                               ,SimpleMatrix goal)
    {
        this(A, B, Q, R, F, goal, 1e-3, 1000);
    }

    /**
     * Evaluate u = -Kx
     * @param x State
     * @return u Control
     */
    public SimpleMatrix getControl(SimpleMatrix x)
    {
        return this.K.mult(x).negative();
    }
}
