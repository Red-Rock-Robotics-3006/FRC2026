package frc.robot.subsystems.shooter.autoaim;

import static java.util.Map.entry;

// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.units.Unit;

import java.util.Map;
import java.util.Map.Entry;
import java.util.TreeMap;

public class InterpolatingTable {

    private InterpolatingTable() {}
                
    public static TreeMap<Double, ShotParameter> table =
        new TreeMap<>(
            Map.ofEntries(
                entry(
                    Double.valueOf(0),
                    new ShotParameter(10, 3000)
                ),
                entry(
                    Double.valueOf(1),
                    new ShotParameter(20, 3500)
                ),
                entry(
                    Double.valueOf(2),
                    new ShotParameter(30, 4500)
                ),
                entry(
                    Double.valueOf(4),
                    new ShotParameter(45, 5000)
                )
                ));

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