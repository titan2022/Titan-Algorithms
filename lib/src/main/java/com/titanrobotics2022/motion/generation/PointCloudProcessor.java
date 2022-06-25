package com.titanrobotics2022.motion.generation;

import com.titanrobotics2022.geometry.geometry2d.Vector2D;
import com.titanrobotics2022.geometry.geometry3d.Vector3D;
import com.titanrobotics2022.motion.generation.rmpflow.RMPRoot;
import com.titanrobotics2022.motion.generation.rmpflow.rmps.CollisionAvoidance;
import java.util.ArrayList;
import java.util.Iterator;
import org.ejml.simple.SimpleMatrix;

public class PointCloudProcessor {
    
    private static final double MAX_HEIGHT = 1.0; // TODO: Configure

    ArrayList<Vector3D> cloud;
    ArrayList<Vector2D> projection;
    
    /**
     * Constructs a PointCloudProcessor object, which transforms a given
     * 3D point cloud to a series of 2D obstacles for avoidance.
     * 
     * @param cloud A 3D point cloud (given with respect to the measuring sensor)
     */
    public PointCloudProcessor(ArrayList<Vector3D> cloud) {
        this.cloud = cloud;
        projection = new ArrayList<>();
        applyThreshold();
        projectTo2D();
    }

    private void applyThreshold() {
        Iterator<Vector3D> it = cloud.iterator();
        while (it.hasNext()) {
            Vector3D vec = it.next();
            if (vec.z < 0 || vec.z > MAX_HEIGHT)
                it.remove();
        }
    }

    /**
     * Formula from https://sensors.myu-group.co.jp/sm_pdf/SM2265.pdf
     */
    private void projectTo2D() {
        for (Vector3D v3 : cloud) {
            double gamma = v3.magnitude();
            double omega = v3.elevationAngle();
            double alpha = v3.azimuthalAngle();
            Vector2D v2 = new Vector2D(
                gamma * Math.cos(omega) * Math.sin(alpha), gamma * Math.cos(omega) * Math.cos(alpha));
            projection.add(v2);
        }
    }

    /**
     * Generate a list of obstacles as CollisionAvoidance RMP instances.
     * 
     * @param root Root node to correspond RMPs with
     * @param name Name of obstacle (each point's name will begin with this name)
     */
    public ArrayList<CollisionAvoidance> generateObstacles(RMPRoot root, String name) {
        ArrayList<CollisionAvoidance> obstacles; = new ArrayList<>();
        int i = 0;
        for (Vector2D vec : projection) {
            String vecName = String.format("%s_%d", name, i++);
            SimpleMatrix center = new SimpleMatrix(1, 2, false, new double[]{ vec.x, vec.y });
            double r = 0.01, epsilon = 0.2, alpha = 1e-5, eta = 0.0; // TODO: configure if necessary
            obstacles.add(new CollisionAvoidance(vecName, root, center, r, epsilon, alpha, eta));
        }
        return obstacles;
    }

    public ArrayList<Vector3D> getCloud() {
        return cloud;
    }
    
    public ArrayList<Vector2D> getProjection() {
        return projection;
    }

}
