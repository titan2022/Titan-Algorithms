package com.titanrobotics2022.geometry.geometry3d;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.apache.commons.math3.util.FastMath;

public class VectorCartesian3DTest {
    private static final double delta = 1e-9;
    @Test
    void getZeroVectorTest()
    {
        VectorCartesian3D z = VectorCartesian3D.ZERO;
        assertEquals(0, z.x);
        assertEquals(0, z.y);
        assertEquals(0, z.z);
    }
    @Test
    public void additionTest() {
        VectorCartesian3D a = new VectorCartesian3D(1, 1, 1);
        VectorCartesian3D b = new VectorCartesian3D(2, 3, 4);
        VectorCartesian3D actual = a.plus(b);
        assertEquals(1 + 2, actual.x, delta);
        assertEquals(1 + 3, actual.y, delta);
        assertEquals(1 + 4, actual.z, delta);
    }
    @Test
    void subtractionTest() {
        VectorCartesian3D a = new VectorCartesian3D(1, 1, 1);
        VectorCartesian3D b = new VectorCartesian3D(2, 3, 4);
        VectorCartesian3D actual = a.minus(b);
        assertEquals(1 - 2, actual.x, delta);
        assertEquals(1 - 3, actual.y, delta);
        assertEquals(1 - 4, actual.z, delta);
    }
    @Test
    void scalarMultiplicationTest() {
        final double scalar = 2.4;
        VectorCartesian3D a = new VectorCartesian3D(2.5, 3.5, 4.5);
        VectorCartesian3D actual = a.scalarMultiply(scalar);
        assertEquals(2.5 * scalar, actual.x, delta);
        assertEquals(3.5 * scalar, actual.y, delta);
        assertEquals(4.5 * scalar, actual.z, delta);
    }
    @Test
    void negationTest()
    {
        VectorCartesian3D a = new VectorCartesian3D(1, -1, 1);
        VectorCartesian3D actual = a.negate();
        assertEquals(-1, actual.x);
        assertEquals(1, actual.y);
        assertEquals(-1, actual.z);
    }
    @Test
    void magnitudeTest()
    {
        VectorCartesian3D a = new VectorCartesian3D(2, 2, 2);
        double actual = a.magnitude();
        assertEquals(Math.sqrt(2 * 2 + 2 * 2 + 2 * 2), actual, delta); 
    }
    @Test
    void magnitudeSquaredTest()
    {
        VectorCartesian3D a = new VectorCartesian3D(2, 2, 2);
        double actual = a.magnitudeSquared();
        assertEquals(2 * 2 + 2 * 2 + 2 * 2, actual, delta); 
    }
    @Test
    void unitVectorTest()
    {
        VectorCartesian3D zeroVector = new VectorCartesian3D(0, 0, 0);
        VectorCartesian3D a = new VectorCartesian3D(-2, 2, -2);
        VectorCartesian3D actual = zeroVector.unitVector();
        assertTrue(VectorCartesian3D.ZERO == actual); // Comparing object references
        actual = a.unitVector();
        double magnitude = FastMath.sqrt(-2 * -2 + 2 * 2 + -2 * -2);                 
        VectorCartesian3D expected = new VectorCartesian3D(-2 / magnitude, 2 / magnitude, -2 / magnitude);
        assertTrue(actual.equals(expected, delta));
    }
    @Test
    void dotProductTest()
    {
        VectorCartesian3D a = new VectorCartesian3D(2, 3, 1);
        VectorCartesian3D b = new VectorCartesian3D(-3, 3, -1);
        double actual = a.dot(b);
        double expected = 2 * -3 + 3 * 3 + 1 * -1;
        assertEquals(expected, actual, delta);
        actual = b.dot(a);
        assertEquals(expected, actual, delta);
    }
    @Test
    void crossProductTest()
    {
        VectorCartesian3D a = new VectorCartesian3D(2, 3, 4);
        VectorCartesian3D b = new VectorCartesian3D(-3, 3, 5);
        VectorCartesian3D actual = a.cross(b);
        assertEquals(3 * 5 - 4 * 3, actual.x, delta);
        assertEquals(-(2 * 5 - 4 * -3), actual.y, delta);
        assertEquals(2 * 3 - 3 * -3, actual.z, delta);
    }
    @Test
    void projectionTest()
    {
        VectorCartesian3D a = new VectorCartesian3D(1, 1, 0);
        VectorCartesian3D b = new VectorCartesian3D(2, 0, 0);
        VectorCartesian3D longerB = new VectorCartesian3D(10, 0, 0);
        VectorCartesian3D actual = a.projectOnto(b);
        assertEquals(1, actual.x, delta);
        assertEquals(0, actual.y, delta);
        actual = a.projectOnto(longerB);
        assertEquals(1, actual.x, delta);
        assertEquals(0, actual.y, delta);
    }
    @Test
    void azimuthalAngleTest()
    {
        VectorCartesian3D positiveX = new VectorCartesian3D(2, 2, 3);
        VectorCartesian3D negativeX = new VectorCartesian3D(-2, 2, 3);
        VectorCartesian3D alongPositiveX = new VectorCartesian3D(2, 0, 0);
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
        VectorCartesian3D a = new VectorCartesian3D(1, 1, 0);
        VectorCartesian3D b = new VectorCartesian3D(1, 0, 0);
        VectorCartesian3D c = new VectorCartesian3D(0, -1, 0);
        double actual = VectorCartesian3D.shortestAngleBetween(a, b);
        assertEquals(Math.PI / 4, actual, delta);
        actual = VectorCartesian3D.shortestAngleBetween(b, a);
        assertEquals(Math.PI / 4, actual, delta);
        actual = VectorCartesian3D.shortestAngleBetween(a, b.negate());
        assertEquals(3 * Math.PI / 4, actual, delta);
        actual = VectorCartesian3D.shortestAngleBetween(a, c);
        assertEquals(3 * Math.PI / 4, actual, delta);
    }
    @Test()
    void specialDoubleTypesTest()
    {
        VectorCartesian3D nan = new VectorCartesian3D(1, 1, Double.NaN);
        VectorCartesian3D posInf = new VectorCartesian3D(1, 1, Double.POSITIVE_INFINITY);
        VectorCartesian3D negInf = new VectorCartesian3D(1, 1, Double.NEGATIVE_INFINITY);
        assertTrue(nan.isNaN());
        assertFalse(nan.isInfinite());
        assertTrue(posInf.isInfinite());
        assertTrue(negInf.isInfinite());
    }
    @Test()
    void dimensionsTest()
    {
        VectorCartesian3D a = new VectorCartesian3D(0, 0, 0);
        assertEquals(3, a.getDimension());
    }
    @Test
    void equalsTest()
    {
        final double x = 1, y = 2;
        VectorCartesian3D ones = new VectorCartesian3D(x, x, x);
        VectorCartesian3D ones2 = new VectorCartesian3D(x, x, x);
        VectorCartesian3D twoes = new VectorCartesian3D(y, y, y);
        assertEquals(true, ones.equals(ones2));
        assertEquals(false, ones.equals(twoes));
    }
    @Test
    void equalsWithToleranceTest()
    {
        final double x = Math.PI, y = Math.E;
        VectorCartesian3D ones = new VectorCartesian3D(x, x, x);
        VectorCartesian3D ones2 = new VectorCartesian3D(x, x, x);
        VectorCartesian3D twoes = new VectorCartesian3D(y, y, y);
        assertEquals(true, ones.equals(ones2, delta));
        assertEquals(false, ones.equals(twoes, delta));
    }
}