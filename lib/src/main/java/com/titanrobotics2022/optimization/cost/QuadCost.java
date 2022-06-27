package com.titanrobotics2022.optimization.cost;

import org.ejml.simple.SimpleMatrix;

public class QuadCost {
    private SimpleMatrix weights, translation;
    public QuadCost(SimpleMatrix weights)
    {
        this.weights = weights;
        this.translation = new SimpleMatrix(weights.numRows() ,1); // Zero vector
    }

    public QuadCost(SimpleMatrix weights, SimpleMatrix translation)
    {
        this.weights = weights;
        this.translation = translation;
    }

    public SimpleMatrix getCostMatrix()
    {
        return weights;
    }

    public SimpleMatrix evalCost(SimpleMatrix input)
    {
        return input.transpose().mult(weights).mult(input.minus(translation));
    }

    public SimpleMatrix evalCostJacobian(SimpleMatrix input)
    {
        return weights.plus(weights.transpose()).mult(input.minus(translation));
    }

    public SimpleMatrix evalCostHessian(SimpleMatrix input)
    {
        return weights.plus(weights.transpose());
    }
}
