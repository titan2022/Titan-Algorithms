package com.titanrobotics2022.geometry.geometry3d;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.apache.commons.math3.util.FastMath;
import org.junit.jupiter.api.Test;

public class VectorCylindricalTest {
    private static final double delta = 1e-9;
    @Test
    void getZeroVectorTest()
    {
        VectorCylindrical z = VectorCylindrical.ZERO;
        assertEquals(0, z.r);
        assertEquals(0, z.theta);
    }
    @Test
    void additionTest() {
        // 45 degree right triangles
        VectorCylindrical a = new VectorCylindrical(FastMath.sqrt(2), Math.PI / 4, 0);
        VectorCylindrical b = new VectorCylindrical(FastMath.sqrt(2), 3 * Math.PI / 4, 0);
        VectorCylindrical actual = a.plus(b);
        assertEquals(1 + 1, actual.r, delta); // Y = 1 + 1, X = 1 - 1
        assertEquals(Math.PI / 2, actual.theta, delta);

        // Adding ZERO vector gives other vector
        VectorCylindrical c = new VectorCylindrical(1, Math.PI, 0);
        actual = VectorCylindrical.ZERO.plus(c);
        assertTrue(actual.equals(c, delta));
    }
    @Test
    void subtractionTest() {
        // 45 degree right triangles
        VectorCylindrical a = new VectorCylindrical(FastMath.sqrt(2), Math.PI / 4, 0);
        VectorCylindrical b = new VectorCylindrical(FastMath.sqrt(2), 3 * Math.PI / 4, 0);
        VectorCylindrical actual = a.minus(b);
        assertEquals(1 + 1, actual.r, delta); // Y = 1 - 1, X = 1 - (-1)
        assertEquals(0, actual.theta, delta);

        // Adding ZERO vector gives other vector
        VectorCylindrical c = new VectorCylindrical(1, Math.PI, 0);
        actual = VectorCylindrical.ZERO.minus(c);
        assertTrue(actual.equals(c.negate(), delta));
    }
    @Test
    void scalarMultiplicationTest() {
        final double scalar = 1.0 / 3.0;
        VectorCylindrical a = new VectorCylindrical(2.5, 3.5, 4.5);
        VectorCylindrical actual = a.scalarMultiply(scalar);
        assertEquals(2.5 * scalar, actual.r, delta);
        assertEquals(4.5 * scalar, actual.z, delta);
        assertEquals(3.5, actual.theta, delta);
    }
    @Test
    void negationTest()
    {
        VectorCylindrical a = new VectorCylindrical(1, Math.PI / 4, 5);
        VectorCylindrical actual = a.negate();
        assertEquals(1, actual.r, delta);
        assertEquals(-5, actual.z, delta);
        assertEquals(Math.PI / 4 + Math.PI, actual.theta);
    }
    @Test
    void magnitudeTest()
    {
        VectorCylindrical a = new VectorCylindrical(2, 2, 3);
        double actual = a.magnitude();
        assertEquals(FastMath.sqrt(2 * 2 + 3 * 3), actual, delta); 
    }
    @Test
    void magnitudeSquaredTest()
    {
        VectorCylindrical a = new VectorCylindrical(2, 2, 3);
        double actual = a.magnitudeSquared();
        assertEquals(2 * 2 + 3 * 3, actual, delta); 
    }
    @Test
    void unitVectorTest()
    {
        VectorCylindrical zeroVector = new VectorCylindrical(0, Math.PI, 0);
        VectorCylindrical actual = zeroVector.unitVector();
        assertTrue(VectorCylindrical.ZERO == actual); // Comparing object references

        VectorCylindrical a = new VectorCylindrical(FastMath.sqrt(2), 2,FastMath.sqrt(2));
        actual = a.unitVector();                 
        assertEquals(FastMath.sqrt(2) / 2.0, actual.r);
        assertEquals(FastMath.sqrt(2) / 2.0, actual.z);
        assertEquals(2, actual.theta);
        assertEquals(1, actual.magnitude(), delta);

        // Negative r
        a = new VectorCylindrical(-FastMath.sqrt(2), 2, -FastMath.sqrt(2));
        actual = a.unitVector();                 
        assertEquals(-FastMath.sqrt(2) / 2.0, actual.r, delta);
        assertEquals(-FastMath.sqrt(2) / 2.0, actual.z, delta);
        assertEquals(2, actual.theta, delta);
        assertEquals(1, actual.magnitude(), delta);
    }
    @Test
    void dotProductTest()
    {
        VectorCylindrical a = new VectorCylindrical(2 * FastMath.sqrt(2), Math.PI / 4, 0);
        VectorCylindrical b = new VectorCylindrical(3 * FastMath.sqrt(2), 3 * Math.PI / 4, 0);
        double actual = a.dot(b);
        double expected = 2 * 3 + -2 * 3;
        assertEquals(expected, actual, delta);
        actual = b.dot(a);
        assertEquals(expected, actual, delta);
    }
    @Test
    void crossProductTest()
    {
        VectorCylindrical a = new VectorCartesian3D(2, 3, 4).toCylindrical();
        VectorCylindrical b = new VectorCartesian3D(-3, 3, 5).toCylindrical();
        VectorCartesian3D actual = a.cross(b).toCartesian3D();
        assertEquals(3 * 5 - 4 * 3, actual.x, delta);
        assertEquals(-(2 * 5 - 4 * -3), actual.y, delta);
        assertEquals(2 * 3 - 3 * -3, actual.z, delta);
    }
    @Test
    void projectionTest()
    {
        VectorCylindrical a = new VectorCylindrical(FastMath.sqrt(2), Math.PI / 4, 0);
        VectorCylindrical b = new VectorCylindrical(2, 0, 0);
        VectorCylindrical longerB = new VectorCylindrical(10, 0, 0);
        VectorCylindrical actual = a.projectOnto(b);
        assertEquals(1, actual.r, delta);
        assertEquals(0, actual.theta, delta);
        actual = a.projectOnto(longerB);
        assertEquals(1, actual.r, delta);
        assertEquals(0, actual.theta, delta);
    }
    @Test
    void azimuthalAngleTest()
    {
        VectorCylindrical positiveX = new VectorCylindrical(2, Math.PI / 4.0, 4);
        VectorCylindrical negativeX = new VectorCylindrical(2, 3.0 * Math.PI / 4.0, 3);
        VectorCylindrical alongPositiveX = new VectorCylindrical(2, 0, 10);
        double actual = positiveX.azimuthalAngle();
        assertEquals(Math.PI / 4.0, actual, delta);
        actual = negativeX.azimuthalAngle();
        assertEquals(3 * Math.PI / 4.0, actual, delta);
        actual = alongPositiveX.azimuthalAngle();
        assertEquals(0, actual, delta);

        VectorCylindrical negativeR = new VectorCylindrical(-2, Math.PI / 4, -10);
        actual = negativeR.azimuthalAngle();
        assertEquals(5.0 * Math.PI / 4.0, actual, delta);
    }
    @Test()
    void specialDoubleTypesTest()
    {
        VectorCylindrical nan = new VectorCylindrical(1, 1, Double.NaN);
        VectorCylindrical posInf = new VectorCylindrical(1, 1, Double.POSITIVE_INFINITY);
        VectorCylindrical negInf = new VectorCylindrical(1, 1, Double.NEGATIVE_INFINITY);
        assertTrue(nan.isNaN());
        assertFalse(nan.isInfinite());
        assertTrue(posInf.isInfinite());
        assertTrue(negInf.isInfinite());
    }
    @Test()
    void dimensionsTest()
    {
        VectorCylindrical a = new VectorCylindrical(0, 0, 0);
        assertEquals(3, a.getDimension());
    }
    @Test
    void equalsTest()
    {
        final double x = 1, y = 2;
        VectorCylindrical ones = new VectorCylindrical(x, x, x);
        VectorCylindrical ones2 = new VectorCylindrical(x, x, x);
        VectorCylindrical twoes = new VectorCylindrical(y, y, y);
        assertTrue(ones.equals(ones2, delta));
        assertFalse(ones.equals(twoes, delta));
    }
    @Test
    void toCartesianTransformTest()
    {
        // Positive z octants
        double r = FastMath.sqrt(1 + 1);
        VectorCartesian3D octantPos1 = new VectorCylindrical(r, Math.PI / 4.0, 1).toCartesian3D();
        VectorCartesian3D octantPos2 = new VectorCylindrical(r, 3 * Math.PI / 4.0, 1).toCartesian3D();
        VectorCartesian3D octantPos3 = new VectorCylindrical(r, 5 * Math.PI / 4.0, 1).toCartesian3D();
        VectorCartesian3D octantPos4 = new VectorCylindrical(r, 7 * Math.PI / 4.0, 1).toCartesian3D();        
        VectorCartesian3D octantPos1Expected = new VectorCartesian3D(1, 1, 1);
        VectorCartesian3D octantPos2Expected = new VectorCartesian3D(-1, 1, 1);
        VectorCartesian3D octantPos3Expected = new VectorCartesian3D(-1, -1, 1);
        VectorCartesian3D octantPos4Expected = new VectorCartesian3D(1, -1, 1);
        
        assertTrue(octantPos1.equals(octantPos1Expected, delta));
        assertTrue(octantPos2.equals(octantPos2Expected, delta));
        assertTrue(octantPos3.equals(octantPos3Expected, delta));
        assertTrue(octantPos4.equals(octantPos4Expected, delta));

        // Negative z octants
        VectorCartesian3D octantNeg1 = new VectorCylindrical(r, Math.PI / 4.0, -1).toCartesian3D();
        VectorCartesian3D octantNeg2 = new VectorCylindrical(r, 3 * Math.PI / 4.0, -1).toCartesian3D();
        VectorCartesian3D octantNeg3 = new VectorCylindrical(r, 5 * Math.PI / 4.0, -1).toCartesian3D();
        VectorCartesian3D octantNeg4 = new VectorCylindrical(r, 7 * Math.PI / 4.0, -1).toCartesian3D();        
        VectorCartesian3D octantNeg1Expected = new VectorCartesian3D(1, 1, -1);
        VectorCartesian3D octantNeg2Expected = new VectorCartesian3D(-1, 1, -1);
        VectorCartesian3D octantNeg3Expected = new VectorCartesian3D(-1, -1, -1);
        VectorCartesian3D octantNeg4Expected = new VectorCartesian3D(1, -1, -1);
        
        assertTrue(octantNeg1.equals(octantNeg1Expected, delta));
        assertTrue(octantNeg2.equals(octantNeg2Expected, delta));
        assertTrue(octantNeg3.equals(octantNeg3Expected, delta));
        assertTrue(octantNeg4.equals(octantNeg4Expected, delta));

        //Edge Cases
        VectorCartesian3D zeroRadians = new VectorCylindrical(1, 0, 0).toCartesian3D();
        assertEquals(1, zeroRadians.x, delta);
        assertEquals(0, zeroRadians.y, delta);

        VectorCartesian3D zeroCartesianVector = new VectorCylindrical(0, 0, 0).toCartesian3D();
        assertEquals(zeroCartesianVector, VectorCartesian3D.ZERO);
        zeroCartesianVector = new VectorCylindrical(0, Math.PI, 0).toCartesian3D();
        assertEquals(zeroCartesianVector, VectorCartesian3D.ZERO);
    }
    @Test
    void toSphericalTest()
    {
        // Positive z octants
        double r = FastMath.sqrt(1 + 1);
        VectorSpherical octantPos1 = new VectorCylindrical(r, Math.PI / 4.0, 1).toSpherical();
        VectorSpherical octantPos2 = new VectorCylindrical(r, 3 * Math.PI / 4.0, 1).toSpherical();
        VectorSpherical octantPos3 = new VectorCylindrical(r, 5 * Math.PI / 4.0, 1).toSpherical();
        VectorSpherical octantPos4 = new VectorCylindrical(r, 7 * Math.PI / 4.0, 1).toSpherical();        
        VectorSpherical octantPos1Expected = new VectorCartesian3D(1, 1, 1).toSpherical();
        VectorSpherical octantPos2Expected = new VectorCartesian3D(-1, 1, 1).toSpherical();
        VectorSpherical octantPos3Expected = new VectorCartesian3D(-1, -1, 1).toSpherical();
        VectorSpherical octantPos4Expected = new VectorCartesian3D(1, -1, 1).toSpherical();
        
        assertTrue(octantPos1.equals(octantPos1Expected, delta));
        assertTrue(octantPos2.equals(octantPos2Expected, delta));
        assertTrue(octantPos3.equals(octantPos3Expected, delta));
        assertTrue(octantPos4.equals(octantPos4Expected, delta));

        // Negative z octants
        VectorSpherical octantNeg1 = new VectorCylindrical(r, Math.PI / 4.0, -1).toSpherical();
        VectorSpherical octantNeg2 = new VectorCylindrical(r, 3 * Math.PI / 4.0, -1).toSpherical();
        VectorSpherical octantNeg3 = new VectorCylindrical(r, 5 * Math.PI / 4.0, -1).toSpherical();
        VectorSpherical octantNeg4 = new VectorCylindrical(r, 7 * Math.PI / 4.0, -1).toSpherical();        
        VectorSpherical octantNeg1Expected = new VectorCartesian3D(1, 1, -1).toSpherical();
        VectorSpherical octantNeg2Expected = new VectorCartesian3D(-1, 1, -1).toSpherical();
        VectorSpherical octantNeg3Expected = new VectorCartesian3D(-1, -1, -1).toSpherical();
        VectorSpherical octantNeg4Expected = new VectorCartesian3D(1, -1, -1).toSpherical();
        
        assertTrue(octantNeg1.equals(octantNeg1Expected, delta));
        assertTrue(octantNeg2.equals(octantNeg2Expected, delta));
        assertTrue(octantNeg3.equals(octantNeg3Expected, delta));
        assertTrue(octantNeg4.equals(octantNeg4Expected, delta));
    }
    @Test
    void equalsWithToleranceTest()
    {
        final double x = 1.55, y = 2.55;
        VectorCylindrical ones = new VectorCylindrical(x, x, x);
        VectorCylindrical ones2 = new VectorCylindrical(x, x, x);
        VectorCylindrical twoes = new VectorCylindrical(y, y, y);
        assertEquals(true, ones.equals(ones2, delta));
        assertEquals(false, ones.equals(twoes, delta));
    }
}
