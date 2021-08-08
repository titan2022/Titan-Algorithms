package com.titanrobotics2022.geometry.linesegment;

import com.titanrobotics2022.Range;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;

public class LineSegment2D {
    private Vector2D start, end;

    public LineSegment2D(Vector2D start, Vector2D end)
    {
        this.start = start;
        this.end = end;
    }

    public Vector2D getStart()
    {
        return start;
    }

    public Vector2D getEnd()
    {
        return end;
    }

    /**
     * Determines if implicit line segment is intersected by the explicit line segment.
     * Colinear and overlapping line segments are considered to be intersecting.
     * 
     * @implNote From https://stackoverflow.com/questions/3838329/how-can-i-check-if-two-segments-intersect
     * 
     * @param otherSegment Line segment to be evaluated with.
     * @param openInterval Whether or not the endpoints of the segment are part of the segment's interval. Open excludes endpoints and closed includes endpoints.
     * @return true for intersecting line segments or overlapping colinear segments based on if interval is open or closed.
     */
    public boolean intersects(LineSegment2D otherSegment, boolean openInterval)
    {
        Range<Double> interval1 = new Range<Double>(Math.min(start.getX(), end.getX()), Math.max(start.getX(), end.getX()));
        Range<Double> interval2 = new Range<Double>(Math.min(otherSegment.getStart().getX(), otherSegment.getEnd().getX()), Math.max(otherSegment.getStart().getX(),otherSegment.getEnd().getX()));

        double xAMin = Math.max(interval1.getMin(), interval2.getMin());
        double xAMax = Math.min(interval1.getMax(), interval2.getMax());
        Range<Double> intervalA = new Range<Double>(xAMin, xAMax);

        if (openInterval)
        {
            if (interval1.getMax() <= interval2.getMin())
                return false;
        }
        else
        {
            if (interval1.getMax() < interval2.getMin())
                return false;
        }

        double line1Slope = (start.getY() - end.getY()) / (start.getX() - end.getX());
        double line2Slope = (otherSegment.getStart().getY() - otherSegment.getEnd().getY()) / (otherSegment.getStart().getX() - otherSegment.getEnd().getX());

        if (line1Slope == line2Slope)
            return false; // Parallel segments

        double line1YIntercept = start.getY() - line1Slope * start.getX();
        double line2YIntercept = otherSegment.getStart().getY() - line2Slope * otherSegment.getStart().getX();

        double Xa = (line2YIntercept - line1YIntercept) / (line1Slope - line2Slope);
        if (intervalA.contains(Xa, openInterval))
            return true;
        else
            return false;
    }
}
