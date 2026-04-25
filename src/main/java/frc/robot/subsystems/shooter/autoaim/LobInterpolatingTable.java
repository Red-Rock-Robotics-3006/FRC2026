package frc.robot.subsystems.shooter.autoaim;

import static java.util.Map.entry;

// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.units.Unit;

import java.util.Map;
import java.util.Map.Entry;
import java.util.TreeMap;

public class LobInterpolatingTable {

    private LobInterpolatingTable() {}

    public static TreeMap<Double, ShotParameter> table = 
        new TreeMap<>(
            Map.ofEntries(
                entry(
                    Double.valueOf(5.13220036503524), 
                    new ShotParameter(44.67, 2200)
                ),
                entry(
                    Double.valueOf(6.36148775420231), 
                    new ShotParameter(44.67, 2600)
                ),
                entry(
                    Double.valueOf(7.9501400162639), 
                    new ShotParameter(44.67, 2850)
                ),
                entry(
                    Double.valueOf(9), 
                    new ShotParameter(44.67, 3100)
                )//,
                // entry(
                //     Double.valueOf(12.3204298138742), 
                //     new ShotParameter(44.67, 3800)
                // ),
                // entry(
                //     Double.valueOf(16), 
                //     new ShotParameter(44.67, 4650)
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