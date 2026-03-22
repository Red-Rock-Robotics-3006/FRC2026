package frc.robot.subsystems.shooter.autoaim;

import static java.util.Map.entry;

import java.util.Map;
import java.util.Map.Entry;
import java.util.TreeMap;

public class FlightTimeLerpTable {
    private FlightTimeLerpTable() {}

    // entry(
    //     distance,
    //     flighttime
    // )
    public static TreeMap<Double, Double> table =
            new TreeMap<>(
                    Map.ofEntries(
                        entry(
                            Double.valueOf(1.4049852287675), 
                            Double.valueOf(0.2)
                        ),
                        entry(
                            Double.valueOf(2.05110564189444), 
                            Double.valueOf(0.25)
                        ),
                        entry(
                            Double.valueOf(2.79029301875713), 
                            Double.valueOf(0.3)
                        ),
                        entry(
                            Double.valueOf(3.42614161315327), 
                            Double.valueOf(0.35)
                        ),
                        entry(
                            Double.valueOf(4.20671927734278), 
                            Double.valueOf(0.4)
                        ),
                        entry(
                            Double.valueOf(4.89260698375057), 
                            Double.valueOf(0.45)
                        ),
                        entry(
                            Double.valueOf(5.42628168818297), 
                            Double.valueOf(0.5)
                        ),
                        entry(
                            Double.valueOf(6.1811784254011), 
                            Double.valueOf(0.55)
                        )
                    ));

    public static Double get(double encoderValue) {
        Entry<Double, Double> ceil = table.ceilingEntry(encoderValue);
        Entry<Double, Double> floor = table.floorEntry(encoderValue);
        if (ceil == null) return floor.getValue();
        if (floor == null) return ceil.getValue();
        if (ceil.getValue().equals(floor.getValue())) return ceil.getValue();
        return Double.valueOf(
                lerp(
                        floor.getValue().doubleValue(),
                        ceil.getValue().doubleValue(),
                        (encoderValue - floor.getKey().doubleValue()) / (ceil.getKey().doubleValue() - floor.getKey().doubleValue())
                )
        );
    }

    private static double lerp(double y1, double y2, double t) {
        return y1 + (t * (y2 - y1));
    }
}
