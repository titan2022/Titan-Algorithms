package com.titanrobotics2022;

public class Range<T extends Comparable<T>> {
    private T min, max;

    public Range(T min, T max)
    {
        this.min = min;
        this.max = max;
    }

    public T getMin()
    {
        return min;
    }

    public void setMin(T min)
    {
        this.min = min;
    }

    public T getMax()
    {
        return max;
    }

    public void setMax(T max)
    {
        this.max = max;
    }

    public boolean contains(T value, boolean openInterval)
    {
        boolean contained = (min.compareTo(value) < 0 && max.compareTo(value) > 0);
        if(openInterval)
            return contained;
        else
            return contained || (min.compareTo(value) == 0) || (max.compareTo(value) == 0);
    }
}
