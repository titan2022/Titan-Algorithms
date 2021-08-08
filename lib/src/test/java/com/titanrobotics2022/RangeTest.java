package com.titanrobotics2022;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

public class RangeTest {
    @Test
    void containsOnOpenIntervalTest()
    {
        Range<Double> a = new Range<Double>(0.0, 5.0);
        assertTrue(a.contains(2.5, true));
        assertFalse(a.contains(0.0, true));
        assertFalse(a.contains(5.0, true));
    }
    @Test
    void containsOnClosedIntervalTest()
    {
        Range<Double> a = new Range<Double>(0.0, 5.0);
        assertTrue(a.contains(2.5, false));
        assertTrue(a.contains(0.0, false));
        assertTrue(a.contains(5.0, false));
    }
    @Test
    void containsWithMultipleTypesTest()
    {
        Range<Integer> a = new Range<Integer>(0, 5);
        assertTrue(a.contains(2, true));
        assertFalse(a.contains(0, true));
        assertFalse(a.contains(5, true));

        Range<Float> b = new Range<Float>((float) 0.0, (float) 5.0);
        assertTrue(b.contains((float) 2.5, true));
        assertFalse(b.contains((float) 0.0, true));
        assertFalse(b.contains((float) 5.0, true));
    }
}
