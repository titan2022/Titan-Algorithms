package com.titanrobotics2022.geometry.geometry2d;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.apache.commons.math3.util.FastMath;

public class VectorCartesian2DTest {
    private static final double delta = 1e-9;
    @Test
    void getZeroVectorTest()
    {
        VectorCartesian2D z = VectorCartesian2D.ZERO;
        assertEquals(0, z.x);
        assertEquals(0, z.y);
    }
    @Test
    void additionTest() {
        VectorCartesian2D a = new VectorCartesian2D(1, 1);
        VectorCartesian2D b = new VectorCartesian2D(2, 3);
        VectorCartesian2D actual = a.plus(b);
        assertEquals(1 + 2, actual.x, delta);
        assertEquals(1 + 3, actual.y, delta);
    }
    @Test
    void subtractionTest() {
        VectorCartesian2D a = new VectorCartesian2D(1, 1);
        VectorCartesian2D b = new VectorCartesian2D(2, 3);
        VectorCartesian2D actual = a.minus(b);
        assertEquals(1 - 2, actual.x, delta);
        assertEquals(1 - 3, actual.y, delta);
    }
    @Test
    void scalarMultiplicationTest() {
        final double scalar = 2.4;
        VectorCartesian2D a = new VectorCartesian2D(2.5, 3.5);
        VectorCartesian2D actual = a.scalarMultiply(scalar);
        assertEquals(2.5 * scalar, actual.x, delta);
        assertEquals(3.5 * scalar, actual.y, delta);
    }
    @Test
    void negationTest()
    {
        VectorCartesian2D a = new VectorCartesian2D(1, -1);
        VectorCartesian2D actual = a.negate();
        assertEquals(-1, actual.x);
        assertEquals(1, actual.y);
    }
    @Test
    void magnitudeTest()
    {
        VectorCartesian2D a = new VectorCartesian2D(2, 2);
        double actual = a.magnitude();
        assertEquals(Math.sqrt(2 * 2 + 2 * 2), actual, delta); 
    }
    @Test
    void magnitudeSquaredTest()
    {
        VectorCartesian2D a = new VectorCartesian2D(2, 2);
        double actual = a.magnitudeSquared();
        assertEquals(2 * 2 + 2 * 2, actual, delta); 
    }
    @Test
    void unitVectorTest()
    {
        VectorCartesian2D zeroVector = new VectorCartesian2D(0, 0);
        VectorCartesian2D a = new VectorCartesian2D(-2, 2);
        VectorCartesian2D actual = zeroVector.unitVector();
        assertTrue(VectorCartesian2D.ZERO == actual); // Comparing object references
        actual = a.unitVector();
        double magnitude = FastMath.sqrt(-2 * -2 + 2 * 2);                 
        VectorCartesian2D expected = new VectorCartesian2D(-2 / magnitude, 2 / magnitude);
        assertTrue(actual.equals(expected, delta));
    }
    @Test
    void dotProductTest()
    {
        VectorCartesian2D a = new VectorCartesian2D(2, 3);
        VectorCartesian2D b = new VectorCartesian2D(-3, 3);
        double actual = a.dot(b);
        double expected = 2 * -3 + 3 * 3;
        assertEquals(expected, actual, delta);
        actual = b.dot(a);
        assertEquals(expected, actual, delta);
    }
    @Test
    void crossProductTest()
    {
        VectorCartesian2D a = new VectorCartesian2D(2, 3);
        VectorCartesian2D b = new VectorCartesian2D(-3, 3);
        double actual = a.cross(b);
        assertEquals(2 * 3 - (-3 * 3), actual, delta); //Magnitude of a 3D cross product of vectors on xy plane
    }
    @Test
    void projectionTest()
    {
        VectorCartesian2D a = new VectorCartesian2D(1, 1);
        VectorCartesian2D b = new VectorCartesian2D(2, 0);
        VectorCartesian2D longerB = new VectorCartesian2D(10, 0);
        VectorCartesian2D actual = a.projectOnto(b);
        assertEquals(1, actual.x, delta);
        assertEquals(0, actual.y, delta);
        actual = a.projectOnto(longerB);
        assertEquals(1, actual.x, delta);
        assertEquals(0, actual.y, delta);
    }
    @Test
    void azimuthalAngleTest()
    {
        VectorCartesian2D positiveX = new VectorCartesian2D(2, 2);
        VectorCartesian2D negativeX = new VectorCartesian2D(-2, 2);
        VectorCartesian2D alongPositiveX = new VectorCartesian2D(2,0);
        double actual = positiveX.azimuthalAngle();
        assertEquals(Math.PI / 4.0, actual, delta);
        actual = negativeX.azimuthalAngle();
        assertEquals(3 * Math.PI / 4.0, actual, delta);
        actual = alongPositiveX.azimuthalAngle();
        assertEquals(0, actual, delta);
    }
    @Test
    void angleBetweenTest()
    {
        VectorCartesian2D a = new VectorCartesian2D(1, 1);
        VectorCartesian2D b = new VectorCartesian2D(1, 0);
        VectorCartesian2D c = new VectorCartesian2D(0, -1);
        double actual = VectorCartesian2D.shortestAngleBetween(a, b);
        assertEquals(Math.PI / 4, actual, delta);
        actual = VectorCartesian2D.shortestAngleBetween(b, a);
        assertEquals(Math.PI / 4, actual, delta);
        actual = VectorCartesian2D.shortestAngleBetween(a, b.negate());
        assertEquals(3 * Math.PI / 4, actual, delta);
        actual = VectorCartesian2D.shortestAngleBetween(a, c);
        assertEquals(3 * Math.PI / 4, actual, delta);
    }
    @Test()
    void specialDoubleTypesTest()
    {
        VectorCartesian2D nan = new VectorCartesian2D(1, Double.NaN);
        VectorCartesian2D posInf = new VectorCartesian2D(1, Double.POSITIVE_INFINITY);
        VectorCartesian2D negInf = new VectorCartesian2D(1, Double.NEGATIVE_INFINITY);
        assertTrue(nan.isNaN());
        assertFalse(nan.isInfinite());
        assertTrue(posInf.isInfinite());
        assertTrue(negInf.isInfinite());
    }
    @Test()
    void dimensionsTest()
    {
        VectorCartesian2D a = new VectorCartesian2D(0, 0);
        assertEquals(2, a.getDimension());
    }
    @Test
    void equalsTest()
    {
        final double x = 1, y = 2;
        VectorCartesian2D ones = new VectorCartesian2D(x, x);
        VectorCartesian2D ones2 = new VectorCartesian2D(x, x);
        VectorCartesian2D twoes = new VectorCartesian2D(y, y);
        assertEquals(true, ones.equals(ones2));
        assertEquals(false, ones.equals(twoes));
    }
    @Test
    void toPolarTransformTest()
    {
        VectorPolar quadrant1 = new VectorCartesian2D(1, 1).toPolar();
        VectorPolar quadrant2 = new VectorCartesian2D(-1, 1).toPolar();
        VectorPolar quadrant3 = new VectorCartesian2D(-1, -1).toPolar();
        VectorPolar quadrant4 = new VectorCartesian2D(1, -1).toPolar();
        double r = FastMath.sqrt(1 + 1);
        VectorPolar quadrant1Expected = new VectorPolar(r, Math.PI / 4.0);
        VectorPolar quadrant2Expected = new VectorPolar(r, 3 * Math.PI / 4.0);
        VectorPolar quadrant3Expected = new VectorPolar(r, 5 * Math.PI / 4.0);
        VectorPolar quadrant4Expected = new VectorPolar(r, 7 * Math.PI / 4.0);
        assertTrue(quadrant1.equals(quadrant1Expected, delta));
        assertTrue(quadrant2.equals(quadrant2Expected, delta));
        assertTrue(quadrant3.equals(quadrant3Expected, delta));
        assertTrue(quadrant4.equals(quadrant4Expected, delta));

        //Edge Cases
        VectorPolar zeroRadians = new VectorCartesian2D(1, 0).toPolar();
        assertEquals(1, zeroRadians.r);
        assertEquals(0, zeroRadians.theta);

        VectorPolar zeroPolarVector = new VectorCartesian2D(0, 0).toPolar();
        assertTrue(zeroPolarVector == VectorPolar.ZERO); // Checking Object References
    }
    @Test
    void equalsWithToleranceTest()
    {
        final double x = 1.55, y = 2.55;
        VectorCartesian2D ones = new VectorCartesian2D(x, x);
        VectorCartesian2D ones2 = new VectorCartesian2D(x, x);
        VectorCartesian2D twoes = new VectorCartesian2D(y, y);
        assertEquals(true, ones.equals(ones2, delta));
        assertEquals(false, ones.equals(twoes, delta));
    }
}
