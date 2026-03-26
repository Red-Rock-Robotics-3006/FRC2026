package frc.robot.subsystems.shooter.autoaim;

import static java.util.Map.entry;

import java.util.Map;
import java.util.Map.Entry;
import java.util.TreeMap;

public class HubInterpolatingTable {

    private HubInterpolatingTable() {}

    public static TreeMap<Double, ShotParameter> table = 
        new TreeMap<>(
            Map.ofEntries(
                entry(
                    Double.valueOf(1.4049852287675), 
                    new ShotParameter(15, 2200)
                ),
                entry(
                    Double.valueOf(2.05110564189444), 
                    new ShotParameter(21, 2300)
                ),
                entry(
                    Double.valueOf(2.79029301875713), 
                    new ShotParameter(23, 2450)
                ),
                entry(
                    Double.valueOf(3.42614161315327), 
                    new ShotParameter(25, 2550)
                ),
                entry(
                    Double.valueOf(4.20671927734278), 
                    new ShotParameter(27, 2650)
                ),
                entry(
                    Double.valueOf(4.89260698375057), 
                    new ShotParameter(29, 2750)
                ),
                entry(
                    Double.valueOf(5.42628168818297), 
                    new ShotParameter(31, 2850)
                ),
                entry(
                    Double.valueOf(6.1811784254011), 
                    new ShotParameter(33, 3000)
                )
            )
        );

    public static ShotParameter get(double distanceToTarget) {
        Entry<Double, ShotParameter> ceil = table.ceilingEntry(distanceToTarget);
        Entry<Double, ShotParameter> floor = table.floorEntry(distanceToTarget);
        if (ceil == null) return floor.getValue();
        if (floor == null) return ceil.getValue();
        if (ceil.getValue().equals(floor.getValue())) return ceil.getValue();
        return floor
            .getValue()
            .interpolate(
                ceil.getValue(),
                (distanceToTarget - floor.getKey()) / (ceil.getKey() - floor.getKey()));
    }
}