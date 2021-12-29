package com.titanrobotics2022.geometry.linesegment;

import com.titanrobotics2022.Range;
import com.titanrobotics2022.geometry.geometry2d.VectorCartesian2D;

public class LineSegment2D {
    private VectorCartesian2D start, end;

    public LineSegment2D(VectorCartesian2D start, VectorCartesian2D end)
    {
        this.start = start;
        this.end = end;
    }

    public VectorCartesian2D getStart()
    {
        return start;
    }

    public VectorCartesian2D getEnd()
    {
        return end;
    }

    /**
     * Determines if implicit line segment is intersected by the explicit line segment.
     * Colinear (parallel) and overlapping line segments are considered to be intersecting.
     * 
     * @implNote From https://stackoverflow.com/questions/3838329/how-can-i-check-if-two-segments-intersect
     * 
     * @param otherSegment Line segment to be evaluated with.
     * @param openInterval Whether or not the endpoints of the segment are part of the segment's interval. Open excludes endpoints and closed includes endpoints.
     * @return true for intersecting line segments or overlapping colinear segments based on if interval is open or closed.
     */
    public boolean intersects(LineSegment2D otherSegment, boolean openInterval)
    {
        Range<Double> interval1 = new Range<Double>(Math.min(start.x, end.x), Math.max(start.x, end.x));
        Range<Double> interval2 = new Range<Double>(Math.min(otherSegment.getStart().x, otherSegment.getEnd().x), Math.max(otherSegment.getStart().x,otherSegment.getEnd().x));

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

        double line1Slope = (start.y - end.y) / (start.x - end.x);
        double line2Slope = (otherSegment.getStart().y - otherSegment.getEnd().y) / (otherSegment.getStart().x - otherSegment.getEnd().x);

        if (line1Slope == line2Slope)
            return false; // Parallel segments

        double line1YIntercept = start.y - line1Slope * start.x;
        double line2YIntercept = otherSegment.getStart().y - line2Slope * otherSegment.getStart().x;

        double Xa = (line2YIntercept - line1YIntercept) / (line1Slope - line2Slope);
        if (intervalA.contains(Xa, openInterval))
            return true;
        else
            return false;
    }
}
