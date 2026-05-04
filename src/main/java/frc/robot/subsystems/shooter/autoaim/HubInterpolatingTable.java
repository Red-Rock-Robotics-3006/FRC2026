package frc.robot.subsystems.shooter.autoaim;

import static java.util.Map.entry;

import java.util.Map;
import java.util.Map.Entry;
import java.util.TreeMap;

public class HubInterpolatingTable {

    private HubInterpolatingTable() {}
    private static int rpmAdjust = -175;//25; //100

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
                    new ShotParameter(37, 2675 + rpmAdjust) //2700
                ),
                entry(
                    Double.valueOf(5.790611301), 
                    new ShotParameter(38, 2775 + rpmAdjust) //2825
                ),
                entry(
                    Double.valueOf(6.319311728), 
                    new ShotParameter(39, 2875 + rpmAdjust) //2900
                ),
                entry(
                    Double.valueOf(6.803354068), 
                    new ShotParameter(39, 2975 + rpmAdjust) //3000
                ),
                entry(
                    Double.valueOf(7.375181189), 
                    new ShotParameter(39, 3075 + rpmAdjust) //3125
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