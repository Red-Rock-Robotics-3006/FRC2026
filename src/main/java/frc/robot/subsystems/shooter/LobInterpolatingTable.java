package frc.robot.subsystems.shooter;

import static java.util.Map.entry;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;

import java.util.Map;
import java.util.Map.Entry;
import java.util.TreeMap;

public class LobInterpolatingTable {

    private LobInterpolatingTable() {}
                
    public static TreeMap<Double, ShotParameter> table =
        new TreeMap<>(
            Map.ofEntries(
                entry(
                    new Double(2),
                    new ShotParameter(32, 32))
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