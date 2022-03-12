package com.titanrobotics2022.demos;

import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;
import javax.swing.JFrame;
import javax.swing.JPanel;

import com.titanrobotics2022.mapping.Path;
import com.titanrobotics2022.motion.generation.rmpflow.RMPRoot;
import com.titanrobotics2022.motion.generation.rmpflow.rmps.PathFollowing;
import com.titanrobotics2022.mapping.LinearSegment;
import com.titanrobotics2022.mapping.Point;

import org.ejml.simple.SimpleMatrix;

import java.util.ArrayList;

public class PathFollowingDemo {

    public PathFollowingDemo() {

        RMPRoot r = new RMPRoot("root");
        SimpleMatrix x = new SimpleMatrix(1, 2, false, new double[] { 5, 21 });
        SimpleMatrix x_dot = new SimpleMatrix(1, 2, false, new double[] { 3, 2 });
        SimpleMatrix x_ddot;
        double v = 5, P = 0.1, I = 0, A = 1, B = 1, K = 1, h = 0.5;

        SimpleMatrix goal = new SimpleMatrix(1, 2, false, new double[] { 90, 65 });
        Path path = new LinearSegment(new Point(x.get(0), x.get(1)), new Point(goal.get(0), goal.get(1)));
        PathFollowing pathFollower = new PathFollowing("Path Following Demo", r, path, v, P, I, A, B, K, h);

        ArrayList<Double> simulationData = new ArrayList<Double>();
        double E = 0.5;
        int MAX_ITER = 10000;
        for (int i = 0; Math.abs(x.minus(goal).normF()) > E && i < MAX_ITER; i++) {
            i++;
            x_ddot = r.solve(x, x_dot);
            System.out.printf("x: (%f, %f)\n", x.get(0), x.get(1));
            System.out.printf("x_dot: (%f, %f)\n", x_dot.get(0), x_dot.get(1));
            System.out.printf("x_ddot: (%f, %f)\n", x_ddot.get(0), x_ddot.get(1));
            double[] newState = solveIntegration(0.02, x_ddot, x_dot, x);
            x_dot.set(1, newState[3]);
            x_dot.set(0, newState[2]);
            x.set(1, newState[1]);
            x.set(0, newState[0]);
            simulationData.add(newState[0]);
            simulationData.add(newState[1]);
            simulationData.add(newState[2]);
            simulationData.add(newState[3]);
            simulationData.add(x_ddot.get(0));
            simulationData.add(x_ddot.get(1));
        }

        JFrame frame = new JFrame("Path Following Demo");
        frame.setSize(500, 500);
        frame.setVisible(true);

        JPanel panel = new JPanel() {
            @Override
            public void paintComponent(Graphics g) {
                super.paintComponent(g);
                Graphics2D g2 = (Graphics2D) g;
                for (int i = 0; i < simulationData.size(); i += 6) {
                    if (i == 0) { // start
                        g2.setColor(Color.red);
                        g2.fillOval((int) (simulationData.get(i) / 1),
                                (int) (simulationData.get(i + 1) / 1), 10, 10);
                    } else {
                        g2.setColor(Color.black);
                        g2.fillOval((int) (simulationData.get(i) / 1),
                                (int) (simulationData.get(i + 1) / 1), 3, 3);
                    }
                }
                g2.setColor(Color.blue);
                g2.fillOval((int) (goal.get(0)),
                        (int) (goal.get(1)), 10, 10);
            }
        };
        frame.add(panel);

    }

    public double getMagnitude(double x, double y) {
        return Math.sqrt(x * x + y * y);
    }

    public static double[] solveIntegration(double deltaT, SimpleMatrix x_ddot, SimpleMatrix x_dot, SimpleMatrix x) {
        double[] state = new double[4];
        state[3] = x_dot.get(1) + x_ddot.get(1) * deltaT;
        state[2] = x_dot.get(0) + x_ddot.get(0) * deltaT;
        state[1] = x.get(1) + .5 * (x_dot.get(1) + state[3]) * deltaT;
        state[0] = x.get(0) + .5 * (x_dot.get(0) + state[2]) * deltaT;
        return state;
    }

}
