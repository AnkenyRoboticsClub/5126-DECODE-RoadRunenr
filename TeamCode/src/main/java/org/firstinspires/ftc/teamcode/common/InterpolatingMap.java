package org.firstinspires.ftc.teamcode.common;

import java.util.TreeMap;

public class InterpolatingMap {
    private final TreeMap<Double, Double> map = new TreeMap<>();

    public void put(double x, double y) {
        map.put(x, y);
    }

    public double get(double x) {
        if (map.containsKey(x)) return map.get(x);

        Double lowerKey = map.floorKey(x);
        Double upperKey = map.ceilingKey(x);

        if (lowerKey == null) return map.get(upperKey);
        if (upperKey == null) return map.get(lowerKey);

        double lowerVal = map.get(lowerKey);
        double upperVal = map.get(upperKey);

        double ratio = (x - lowerKey) / (upperKey - lowerKey);

        return lowerVal + ratio * (upperVal - lowerVal);
    }
}
