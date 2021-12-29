package com.titanrobotics2022.geometry.geometry3d;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.apache.commons.math3.util.FastMath;
import org.junit.jupiter.api.Test;

public class VectorSphericalTest {
    private static final double delta = 1e-9;
    @Test
    void getZeroVectorTest()
    {
        VectorSpherical z = VectorSpherical.ZERO;
        assertEquals(0, z.rho);
        assertEquals(0, z.theta);
        assertEquals(0, z.phi);
    }
    @Test
    void additionTest() {
        // 45 degree right triangles
        VectorSpherical a = new VectorSpherical(FastMath.sqrt(2), Math.PI / 4, Math.PI / 4);
        VectorSpherical b = new VectorSpherical(FastMath.sqrt(2), Math.PI / 4, 3 * Math.PI / 4);
        VectorSpherical actual = a.plus(b);
        assertEquals(1 + 1, actual.rho, delta); // Y = 1 + 1, Z = 1 - 1
        assertEquals(Math.PI / 4, actual.theta, delta);
        assertEquals(Math.PI / 2, actual.phi, delta);

        // Adding ZERO vector gives other vector
        VectorSpherical c = new VectorSpherical(1, Math.PI, Math.PI / 2);
        actual = VectorSpherical.ZERO.plus(c);
        assertTrue(actual.equals(c, delta));
    }
    @Test
    void subtractionTest() {
        // 45 degree right triangles
        VectorSpherical a = new VectorSpherical(FastMath.sqrt(2), Math.PI / 4, Math.PI / 4);
        VectorSpherical b = new VectorSpherical(FastMath.sqrt(2), Math.PI / 4, 3 * Math.PI / 4);
        VectorSpherical actual = a.minus(b);
        assertEquals(1 + 1, actual.rho, delta); // Y = 1 - 1, Z = 1 - (-1)
        assertEquals(0, actual.theta, delta);
        assertEquals(0, actual.phi, delta);

        // Adding ZERO vector gives other vector
        VectorSpherical c = new VectorSpherical(1, Math.PI, Math.PI / 2);
        actual = VectorSpherical.ZERO.minus(c);
        assertTrue(actual.equals(c.negate(), delta));
    }
    @Test
    void scalarMultiplicationTest() {
        final double scalar = 1.0 / 3.0;
        VectorSpherical a = new VectorSpherical(5, Math.PI, Math.PI / 4);
        VectorSpherical actual = a.scalarMultiply(scalar);
        assertEquals(5 * scalar, actual.rho, delta);
        assertEquals(Math.PI, actual.theta, delta);
        assertEquals(Math.PI / 4, actual.phi, delta);
    }
    @Test
    void negationTest()
    {
        VectorSpherical a = new VectorSpherical(1, Math.PI / 4, Math.PI / 4);
        VectorSpherical actual = a.negate();
        assertEquals(1, actual.rho, delta);
        assertEquals(Math.PI + Math.PI / 4, actual.theta, delta);
        assertEquals(3 * Math.PI / 4, actual.phi);
    }
    @Test
    void magnitudeTest()
    {
        VectorSpherical a = new VectorSpherical(2, Math.PI, Math.PI / 4);
        double actual = a.magnitude();
        assertEquals(2, actual, delta); 
    }
    @Test
    void magnitudeSquaredTest()
    {
        VectorSpherical a = new VectorSpherical(2, Math.PI, Math.PI / 4);
        double actual = a.magnitudeSquared();
        assertEquals(2 * 2, actual, delta); 
    }
    @Test
    void unitVectorTest()
    {
        VectorSpherical zeroVector = new VectorSpherical(0, Math.PI, 0);
        VectorSpherical actual = zeroVector.unitVector();
        assertTrue(VectorSpherical.ZERO == actual); // Comparing object references

        VectorSpherical a = new VectorSpherical(4, Math.PI , Math.PI / 4);
        actual = a.unitVector();                 
        assertEquals(1, actual.rho, delta);
        assertEquals(Math.PI, actual.theta, delta);
        assertEquals(Math.PI / 4, actual.phi, delta);
        assertEquals(1, actual.magnitude(), delta);
    }
    @Test
    void dotProductTest()
    {
        VectorSpherical a = new VectorSpherical(2 * FastMath.sqrt(2), Math.PI / 4, Math.PI / 2);
        VectorSpherical b = new VectorSpherical(3 * FastMath.sqrt(2), 3 * Math.PI / 4, Math.PI / 2);
        double actual = a.dot(b);
        double expected = 2 * 3 + -2 * 3;
        assertEquals(expected, actual, delta);
        actual = b.dot(a);
        assertEquals(expected, actual, delta);
    }
    @Test
    void crossProductTest()
    {
        VectorSpherical a = new VectorCartesian3D(2, 3, 4).toSpherical();
        VectorSpherical b = new VectorCartesian3D(-3, 3, 5).toSpherical();
        VectorCartesian3D actual = a.cross(b).toCartesian3D();
        assertEquals(3 * 5 - 4 * 3, actual.x, delta);
        assertEquals(-(2 * 5 - 4 * -3), actual.y, delta);
        assertEquals(2 * 3 - 3 * -3, actual.z, delta);
    }
    @Test
    void projectionTest()
    {
        VectorSpherical a = new VectorSpherical(FastMath.sqrt(2), 0, Math.PI / 4);
        VectorSpherical b = new VectorSpherical(2, 0, Math.PI / 2);
        VectorSpherical longerB = new VectorSpherical(10, 0, Math.PI / 2);
        VectorSpherical actual = a.projectOnto(b);
        assertEquals(1, actual.rho, delta);
        assertEquals(0, actual.theta, delta);
        actual = a.projectOnto(longerB);
        assertEquals(1, actual.rho, delta);
        assertEquals(0, actual.theta, delta);
    }
    @Test
    void azimuthalAngleTest()
    {
        VectorSpherical positiveX = new VectorSpherical(2, Math.PI / 4, Math.PI / 2);
        VectorSpherical negativeX = new VectorSpherical(2, 3 * Math.PI / 4, Math.PI / 2);
        VectorSpherical alongPositiveX = new VectorSpherical(2, 0, Math.PI / 4);
        double actual = positiveX.azimuthalAngle();
        assertEquals(Math.PI / 4.0, actual, delta);
        actual = negativeX.azimuthalAngle();
        assertEquals(3 * Math.PI / 4.0, actual, delta);
        actual = alongPositiveX.azimuthalAngle();
        assertEquals(0, actual, delta);
    }
    @Test()
    void specialDoubleTypesTest()
    {
        VectorSpherical nan = new VectorSpherical(1, 1, Double.NaN);
        VectorSpherical posInf = new VectorSpherical(1, 1, Double.POSITIVE_INFINITY);
        VectorSpherical negInf = new VectorSpherical(1, 1, Double.NEGATIVE_INFINITY);
        assertTrue(nan.isNaN());
        assertFalse(nan.isInfinite());
        assertTrue(posInf.isInfinite());
        assertTrue(negInf.isInfinite());
    }
    @Test()
    void dimensionsTest()
    {
        VectorSpherical a = new VectorSpherical(0, 0, 0);
        assertEquals(3, a.getDimension());
    }
    @Test
    void equalsTest()
    {
        final double x = 1, y = 2;
        VectorSpherical ones = new VectorSpherical(x, x, x);
        VectorSpherical ones2 = new VectorSpherical(x, x, x);
        VectorSpherical twoes = new VectorSpherical(y, y, y);
        assertTrue(ones.equals(ones2, delta));
        assertFalse(ones.equals(twoes, delta));
    }
    @Test
    void toCartesianTransformTest()
    {
        // Positive z octants
        double r = FastMath.sqrt(1 + 1 + 1);
        double phi = Math.PI / 2 - Math.atan(1 / Math.sqrt(2));
        VectorCartesian3D octantPos1 = new VectorSpherical(r, Math.PI / 4, phi).toCartesian3D();
        VectorCartesian3D octantPos2 = new VectorSpherical(r, 3 * Math.PI / 4, phi).toCartesian3D();
        VectorCartesian3D octantPos3 = new VectorSpherical(r, 5 * Math.PI / 4, phi).toCartesian3D();
        VectorCartesian3D octantPos4 = new VectorSpherical(r, 7 * Math.PI / 4, phi).toCartesian3D();        
        VectorCartesian3D octantPos1Expected = new VectorCartesian3D(1, 1, 1);
        VectorCartesian3D octantPos2Expected = new VectorCartesian3D(-1, 1, 1);
        VectorCartesian3D octantPos3Expected = new VectorCartesian3D(-1, -1, 1);
        VectorCartesian3D octantPos4Expected = new VectorCartesian3D(1, -1, 1);
        
        assertTrue(octantPos1.equals(octantPos1Expected, delta));
        assertTrue(octantPos2.equals(octantPos2Expected, delta));
        assertTrue(octantPos3.equals(octantPos3Expected, delta));
        assertTrue(octantPos4.equals(octantPos4Expected, delta));

        // Negative z octants
        VectorCartesian3D octantNeg1 = new VectorSpherical(r, Math.PI / 4.0, Math.PI - phi).toCartesian3D();
        VectorCartesian3D octantNeg2 = new VectorSpherical(r, 3 * Math.PI / 4.0, Math.PI - phi).toCartesian3D();
        VectorCartesian3D octantNeg3 = new VectorSpherical(r, 5 * Math.PI / 4.0, Math.PI - phi).toCartesian3D();
        VectorCartesian3D octantNeg4 = new VectorSpherical(r, 7 * Math.PI / 4.0, Math.PI - phi).toCartesian3D();        
        VectorCartesian3D octantNeg1Expected = new VectorCartesian3D(1, 1, -1);
        VectorCartesian3D octantNeg2Expected = new VectorCartesian3D(-1, 1, -1);
        VectorCartesian3D octantNeg3Expected = new VectorCartesian3D(-1, -1, -1);
        VectorCartesian3D octantNeg4Expected = new VectorCartesian3D(1, -1, -1);
        
        assertTrue(octantNeg1.equals(octantNeg1Expected, delta));
        assertTrue(octantNeg2.equals(octantNeg2Expected, delta));
        assertTrue(octantNeg3.equals(octantNeg3Expected, delta));
        assertTrue(octantNeg4.equals(octantNeg4Expected, delta));

        //Edge Cases
        VectorCartesian3D zeroRadians = new VectorSpherical(1, 0, Math.PI / 2).toCartesian3D();
        assertEquals(1, zeroRadians.x, delta);
        assertEquals(0, zeroRadians.y, delta);

        VectorCartesian3D zeroCartesianVector = new VectorSpherical(0, 0, 0).toCartesian3D();
        assertEquals(zeroCartesianVector, VectorCartesian3D.ZERO);
        zeroCartesianVector = new VectorSpherical(0, Math.PI, 0).toCartesian3D();
        assertEquals(zeroCartesianVector, VectorCartesian3D.ZERO);
    }
    @Test
    void toCylindricalTest()
    {
        // Positive z octants
        double r = FastMath.sqrt(1 + 1 + 1);
        double phi = Math.PI / 2 - Math.atan(1 / Math.sqrt(2));
        VectorCylindrical octantPos1 = new VectorSpherical(r, Math.PI / 4, phi).toCylindrical();
        VectorCylindrical octantPos2 = new VectorSpherical(r, 3 * Math.PI / 4, phi).toCylindrical();
        VectorCylindrical octantPos3 = new VectorSpherical(r, 5 * Math.PI / 4, phi).toCylindrical();
        VectorCylindrical octantPos4 = new VectorSpherical(r, 7 * Math.PI / 4, phi).toCylindrical();        
        VectorCylindrical octantPos1Expected = new VectorCartesian3D(1, 1, 1).toCylindrical();
        VectorCylindrical octantPos2Expected = new VectorCartesian3D(-1, 1, 1).toCylindrical();
        VectorCylindrical octantPos3Expected = new VectorCartesian3D(-1, -1, 1).toCylindrical();
        VectorCylindrical octantPos4Expected = new VectorCartesian3D(1, -1, 1).toCylindrical();
        
        assertTrue(octantPos1.equals(octantPos1Expected, delta));
        assertTrue(octantPos2.equals(octantPos2Expected, delta));
        assertTrue(octantPos3.equals(octantPos3Expected, delta));
        assertTrue(octantPos4.equals(octantPos4Expected, delta));

        // Negative z octants
        VectorCylindrical octantNeg1 = new VectorSpherical(r, Math.PI / 4.0, Math.PI - phi).toCylindrical();
        VectorCylindrical octantNeg2 = new VectorSpherical(r, 3 * Math.PI / 4.0, Math.PI - phi).toCylindrical();
        VectorCylindrical octantNeg3 = new VectorSpherical(r, 5 * Math.PI / 4.0, Math.PI - phi).toCylindrical();
        VectorCylindrical octantNeg4 = new VectorSpherical(r, 7 * Math.PI / 4.0, Math.PI - phi).toCylindrical();        
        VectorCylindrical octantNeg1Expected = new VectorCartesian3D(1, 1, -1).toCylindrical();
        VectorCylindrical octantNeg2Expected = new VectorCartesian3D(-1, 1, -1).toCylindrical();
        VectorCylindrical octantNeg3Expected = new VectorCartesian3D(-1, -1, -1).toCylindrical();
        VectorCylindrical octantNeg4Expected = new VectorCartesian3D(1, -1, -1).toCylindrical();
        
        assertTrue(octantNeg1.equals(octantNeg1Expected, delta));
        assertTrue(octantNeg2.equals(octantNeg2Expected, delta));
        assertTrue(octantNeg3.equals(octantNeg3Expected, delta));
        assertTrue(octantNeg4.equals(octantNeg4Expected, delta));
    }
    @Test
    void equalsWithToleranceTest()
    {
        final double x = 1.55, y = 2.55;
        VectorSpherical ones = new VectorSpherical(x, x, x);
        VectorSpherical ones2 = new VectorSpherical(x, x, x);
        VectorSpherical twoes = new VectorSpherical(y, y, y);
        assertEquals(true, ones.equals(ones2, delta));
        assertEquals(false, ones.equals(twoes, delta));
    }
}
