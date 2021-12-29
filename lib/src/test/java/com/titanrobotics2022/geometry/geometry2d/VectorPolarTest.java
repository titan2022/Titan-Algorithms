package com.titanrobotics2022.geometry.geometry2d;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.apache.commons.math3.util.FastMath;

public class VectorPolarTest {
    private static final double delta = 1e-9;
    @Test
    void getZeroVectorTest()
    {
        VectorPolar z = VectorPolar.ZERO;
        assertEquals(0, z.r);
        assertEquals(0, z.theta);
    }
    @Test
    void additionTest() {
        // 45 degree right triangles
        VectorPolar a = new VectorPolar(FastMath.sqrt(2), Math.PI / 4);
        VectorPolar b = new VectorPolar(FastMath.sqrt(2), 3 * Math.PI / 4);
        VectorPolar actual = a.plus(b);
        assertEquals(1 + 1, actual.r, delta); // Y = 1 + 1, X = 1 - 1
        assertEquals(Math.PI / 2, actual.theta, delta);

        // Adding ZERO vector gives other vector
        VectorPolar c = new VectorPolar(1, Math.PI);
        actual = VectorPolar.ZERO.plus(c);
        assertTrue(actual.equals(c, delta));
    }
    @Test
    void subtractionTest() {
        // 45 degree right triangles
        VectorPolar a = new VectorPolar(FastMath.sqrt(2), Math.PI / 4);
        VectorPolar b = new VectorPolar(FastMath.sqrt(2), 3 * Math.PI / 4);
        VectorPolar actual = a.minus(b);
        assertEquals(1 + 1, actual.r, delta); // Y = 1 - 1, X = 1 - (-1)
        assertEquals(0, actual.theta, delta);

        // Adding ZERO vector gives other vector
        VectorPolar c = new VectorPolar(1, Math.PI);
        actual = VectorPolar.ZERO.minus(c);
        assertTrue(actual.equals(c.negate(), delta));
    }
    @Test
    void scalarMultiplicationTest() {
        final double scalar = 1.0 / 3.0;
        VectorPolar a = new VectorPolar(2.5, 3.5);
        VectorPolar actual = a.scalarMultiply(scalar);
        assertEquals(2.5 * scalar, actual.r, delta);
        assertEquals(3.5, actual.theta, delta);
    }
    @Test
    void negationTest()
    {
        VectorPolar a = new VectorPolar(1, Math.PI / 4);
        VectorPolar actual = a.negate();
        assertEquals(1, actual.r, delta);
        assertEquals(Math.PI / 4 + Math.PI, actual.theta);
    }
    @Test
    void magnitudeTest()
    {
        VectorPolar a = new VectorPolar(2, 2);
        double actual = a.magnitude();
        assertEquals(2, actual, delta); 
    }
    @Test
    void magnitudeSquaredTest()
    {
        VectorPolar a = new VectorPolar(2, 2);
        double actual = a.magnitudeSquared();
        assertEquals(2 * 2, actual, delta); 
    }
    @Test
    void unitVectorTest()
    {
        VectorPolar zeroVector = new VectorPolar(0, Math.PI);
        VectorPolar actual = zeroVector.unitVector();
        assertTrue(VectorPolar.ZERO == actual); // Comparing object references
        double magnitude = 5;
        VectorPolar a = new VectorPolar(magnitude, 2);
        actual = a.unitVector();                 
        assertEquals(1, actual.r);
        assertEquals(2, actual.theta);

        // Negative r
        magnitude = -5;
        a = new VectorPolar(magnitude, 2);
        actual = a.unitVector();                 
        assertEquals(-1, actual.r);
        assertEquals(2, actual.theta);
    }
    @Test
    void dotProductTest()
    {
        VectorPolar a = new VectorPolar(2 * FastMath.sqrt(2), Math.PI / 4);
        VectorPolar b = new VectorPolar(3 * FastMath.sqrt(2), 3 * Math.PI / 4);
        double actual = a.dot(b);
        double expected = 2 * 3 + -2 * 3;
        assertEquals(expected, actual, delta);
        actual = b.dot(a);
        assertEquals(expected, actual, delta);
    }
    @Test
    void crossProductTest()
    {
        VectorPolar a = new VectorCartesian2D(2, 3).toPolar();
        VectorPolar b = new VectorCartesian2D(-3, 3).toPolar();
        double actual = a.cross(b);
        assertEquals(2 * 3 - (-3 * 3), actual, delta); //Magnitude of a 3D cross product of vectors on xy plane
    }
    @Test
    void projectionTest()
    {
        VectorPolar a = new VectorPolar(FastMath.sqrt(2), Math.PI / 4);
        VectorPolar b = new VectorPolar(2, 0);
        VectorPolar longerB = new VectorPolar(10, 0);
        VectorPolar actual = a.projectOnto(b);
        assertEquals(1, actual.r, delta);
        assertEquals(0, actual.theta, delta);
        actual = a.projectOnto(longerB);
        assertEquals(1, actual.r, delta);
        assertEquals(0, actual.theta, delta);
    }
    @Test
    void azimuthalAngleTest()
    {
        VectorPolar positiveX = new VectorPolar(2, Math.PI / 4.0);
        VectorPolar negativeX = new VectorPolar(2, 3.0 * Math.PI / 4.0);
        VectorPolar alongPositiveX = new VectorPolar(2,0);
        double actual = positiveX.azimuthalAngle();
        assertEquals(Math.PI / 4.0, actual, delta);
        actual = negativeX.azimuthalAngle();
        assertEquals(3 * Math.PI / 4.0, actual, delta);
        actual = alongPositiveX.azimuthalAngle();
        assertEquals(0, actual, delta);

        VectorPolar negativeR = new VectorPolar(-2, Math.PI / 4);
        actual = negativeR.azimuthalAngle();
        assertEquals(5.0 * Math.PI / 4.0, actual, delta);
    }
    @Test()
    void specialDoubleTypesTest()
    {
        VectorPolar nan = new VectorPolar(1, Double.NaN);
        VectorPolar posInf = new VectorPolar(1, Double.POSITIVE_INFINITY);
        VectorPolar negInf = new VectorPolar(1, Double.NEGATIVE_INFINITY);
        assertTrue(nan.isNaN());
        assertFalse(nan.isInfinite());
        assertTrue(posInf.isInfinite());
        assertTrue(negInf.isInfinite());
    }
    @Test()
    void dimensionsTest()
    {
        VectorPolar a = new VectorPolar(0, 0);
        assertEquals(2, a.getDimension());
    }
    @Test
    void equalsTest()
    {
        final double x = 1, y = 2;
        VectorPolar ones = new VectorPolar(x, x);
        VectorPolar ones2 = new VectorPolar(x, x);
        VectorPolar twoes = new VectorPolar(y, y);
        assertTrue(ones.equals(ones2, delta));
        assertFalse(ones.equals(twoes, delta));

        VectorPolar positiveR = new VectorPolar(1, Math.PI / 4.0);
        VectorPolar negativeR = new VectorPolar(-1, (Math.PI / 4.0) + Math.PI);
        assertTrue(positiveR.equals(negativeR, delta));
    }
    @Test
    void toCartesianTransformTest()
    {
        double r = FastMath.sqrt(1 + 1);
        VectorCartesian2D quadrant1 = new VectorPolar(r, Math.PI / 4.0).toCartesian2D();
        VectorCartesian2D quadrant2 = new VectorPolar(r, 3 * Math.PI / 4.0).toCartesian2D();
        VectorCartesian2D quadrant3 = new VectorPolar(r, 5 * Math.PI / 4.0).toCartesian2D();
        VectorCartesian2D quadrant4 = new VectorPolar(r, 7 * Math.PI / 4.0).toCartesian2D();        
        VectorCartesian2D quadrant1Expected = new VectorCartesian2D(1, 1);
        VectorCartesian2D quadrant2Expected = new VectorCartesian2D(-1, 1);
        VectorCartesian2D quadrant3Expected = new VectorCartesian2D(-1, -1);
        VectorCartesian2D quadrant4Expected = new VectorCartesian2D(1, -1);
        
        assertTrue(quadrant1.equals(quadrant1Expected, delta));
        assertTrue(quadrant2.equals(quadrant2Expected, delta));
        assertTrue(quadrant3.equals(quadrant3Expected, delta));
        assertTrue(quadrant4.equals(quadrant4Expected, delta));

        //Edge Cases
        VectorCartesian2D zeroRadians = new VectorPolar(1, 0).toCartesian2D();
        assertEquals(1, zeroRadians.x, delta);
        assertEquals(0, zeroRadians.y, delta);

        VectorCartesian2D zeroCartesianVector = new VectorPolar(0, 0).toCartesian2D();
        assertTrue(zeroCartesianVector == VectorCartesian2D.ZERO); // Checking Object References
        zeroCartesianVector = new VectorPolar(0, Math.PI).toCartesian2D();
        assertTrue(zeroCartesianVector == VectorCartesian2D.ZERO);
    }
    @Test
    void equalsWithToleranceTest()
    {
        final double x = 1.55, y = 2.55;
        VectorPolar ones = new VectorPolar(x, x);
        VectorPolar ones2 = new VectorPolar(x, x);
        VectorPolar twoes = new VectorPolar(y, y);
        assertEquals(true, ones.equals(ones2, delta));
        assertEquals(false, ones.equals(twoes, delta));
    }
}
