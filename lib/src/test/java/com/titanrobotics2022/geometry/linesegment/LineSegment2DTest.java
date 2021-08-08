package com.titanrobotics2022.geometry.linesegment;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;

public class LineSegment2DTest {
    private static final double delta = 1e-9;
    
    /*
     * Proper intersections contain no colinear points.
     * Improper intersections contain colinear points.
     */
    @Test
    void properIntersectionsTest()
    {
        LineSegment2D seg1 = new LineSegment2D(new Vector2D(0, 0), new Vector2D(1, 1));
        LineSegment2D seg2 = new LineSegment2D(new Vector2D(0, 1), new Vector2D(1, 0));

        assertTrue(seg1.intersects(seg2, true));
        assertTrue(seg1.intersects(seg2, false));
    }
    @Test
    void improperIntersectionsTest()
    {
        LineSegment2D seg1 = new LineSegment2D(new Vector2D(0, 0), new Vector2D(1, 1));
        LineSegment2D seg2 = new LineSegment2D(new Vector2D(0, 0), new Vector2D(1, 0));

        assertFalse(seg1.intersects(seg2, true));
        assertTrue(seg1.intersects(seg2, false));

        seg1 = new LineSegment2D(new Vector2D(0, 0), new Vector2D(1, 1));
        seg2 = new LineSegment2D(new Vector2D(.5, .5), new Vector2D(1, 0));

        assertFalse(seg1.intersects(seg2, true));
        assertTrue(seg1.intersects(seg2, false));
    }
    @Test
    void notIntersectingTesting()
    {
        LineSegment2D seg1 = new LineSegment2D(new Vector2D(0, 0), new Vector2D(-1, -1));
        LineSegment2D seg2 = new LineSegment2D(new Vector2D(0, 1), new Vector2D(1, 0));

        assertFalse(seg1.intersects(seg2, true));
        assertFalse(seg1.intersects(seg2, false));
    }
    /**
     * Overlapping and colinear linesegments are defined as intersecting: {@link LineSegment2D#intersects(LineSegment2D, boolean)}
     */
    @Test // TODO: Research and implement intersections to include code for parallel intersections. Refer to : https://stackoverflow.com/a/3842240
    void colinearIntersectionsTest()
    {
        LineSegment2D seg1 = new LineSegment2D(new Vector2D(0, 0), new Vector2D(1, 1));
        LineSegment2D seg2 = new LineSegment2D(new Vector2D(0, 0), new Vector2D(2, 2));

        //assertTrue(seg1.intersects(seg2, true));
        //assertTrue(seg1.intersects(seg2, false));

        seg1 = new LineSegment2D(new Vector2D(0, 0), new Vector2D(1, 1));
        seg2 = new LineSegment2D(new Vector2D(1, 1), new Vector2D(2, 2));

        //assertFalse(seg1.intersects(seg2, true));
        //assertTrue(seg1.intersects(seg2, false));
    }
}
