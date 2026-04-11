package frc.robot.subsystems.shooter.autoaim;

import static java.util.Map.entry;

import java.util.Map;
import java.util.Map.Entry;
import java.util.TreeMap;

public class HubInterpolatingTable {

    private HubInterpolatingTable() {}
    private static int rpmAdjust = 100; //100

    public static TreeMap<Double, ShotParameter> table = 
        new TreeMap<>(
            Map.ofEntries(
                entry(
                    Double.valueOf(1.340873257), 
                    new ShotParameter(15, 2050 + rpmAdjust)
                ),
                entry(
                    Double.valueOf(1.876275382), 
                    new ShotParameter(22, 2075 + rpmAdjust)
                ),
                entry(
                    Double.valueOf(2.319276105), 
                    new ShotParameter(25, 2125 + rpmAdjust)
                ),
                entry(
                    Double.valueOf(2.80138351), 
                    new ShotParameter(28, 2200 + rpmAdjust)
                ),
                entry(
                    Double.valueOf(3.341286443), 
                    new ShotParameter(31, 2275 + rpmAdjust)
                ),
                entry(
                    Double.valueOf(3.751998681), 
                    new ShotParameter(34, 2375 + rpmAdjust)
                ),
                entry(
                    Double.valueOf(4.363595402), 
                    new ShotParameter(35, 2475 + rpmAdjust)
                ),
                entry(
                    Double.valueOf(4.803119665), 
                    new ShotParameter(36, 2575 + rpmAdjust)
                ),
                entry(
                    Double.valueOf(5.360072962), 
                    new ShotParameter(37, 2700 + rpmAdjust)
                ),
                entry(
                    Double.valueOf(5.790611301), 
                    new ShotParameter(38, 2825 + rpmAdjust)
                ),
                entry(
                    Double.valueOf(6.319311728), 
                    new ShotParameter(39, 2900 + rpmAdjust)
                ),
                entry(
                    Double.valueOf(6.803354068), 
                    new ShotParameter(39, 3000 + rpmAdjust)
                ),
                entry(
                    Double.valueOf(7.375181189), 
                    new ShotParameter(39, 3125 + rpmAdjust)
                )
                // entry(
                //     Double.valueOf(1.4049852287675), 
                //     new ShotParameter(15, 2200 + rpmAdjust)
                // ),
                // entry(
                //     Double.valueOf(2.05110564189444), 
                //     new ShotParameter(21, 2300 + rpmAdjust)
                // ),
                // entry(
                //     Double.valueOf(2.79029301875713), 
                //     new ShotParameter(23, 2450 + rpmAdjust)
                // ),
                // entry(
                //     Double.valueOf(3.42614161315327), 
                //     new ShotParameter(25, 2550 + rpmAdjust)
                // ),
                // entry(
                //     Double.valueOf(4.20671927734278), 
                //     new ShotParameter(27, 2650 + rpmAdjust)
                // ),
                // entry(
                //     Double.valueOf(4.89260698375057), 
                //     new ShotParameter(29, 2750 + rpmAdjust)
                // ),
                // entry(
                //     Double.valueOf(5.42628168818297), 
                //     new ShotParameter(31, 2850 + rpmAdjust)
                // ),
                // entry(
                //     Double.valueOf(6.1811784254011), 
                //     new ShotParameter(33, 3000 + rpmAdjust)
                // )
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