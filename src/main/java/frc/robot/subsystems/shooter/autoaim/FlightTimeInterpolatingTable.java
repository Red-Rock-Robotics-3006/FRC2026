package frc.robot.subsystems.shooter.autoaim;

import static java.util.Map.entry;

import java.util.Map;
import java.util.Map.Entry;
import java.util.TreeMap;

public class FlightTimeInterpolatingTable {
    private FlightTimeInterpolatingTable() {}

    public static TreeMap<Double, Double> table = 
        new TreeMap<>(
            Map.ofEntries(
                entry(
                    Double.valueOf(1.507975), 
                    Double.valueOf(58d / 60d)
                ),
                entry(
                    Double.valueOf(3.00836), 
                    Double.valueOf(58d / 60d)
                ),
                entry(
                    Double.valueOf(4.490183), 
                    Double.valueOf(1 + (1d / 60d))
                ),
                entry(
                    Double.valueOf(6.000448), 
                    Double.valueOf(1 + (10d / 60d))
                ),
                entry(
                    Double.valueOf(7.2848), 
                    Double.valueOf(1 + (16d / 60d))
                )
            )
        );

    // entry(
    //     distance,
    //     flighttime
    // )
    // public static TreeMap<Double, Double> table =
    //         new TreeMap<>(
    //                 Map.ofEntries(

    //                 // 1.415676353
    //                 // 2.007647272
    //                 // 2.811736268
    //                 // 3.495614464
    //                 // 4.258711651
    //                 // 4.844127245
    //                 // 5.375485779
    //                 // 6.221875559
                    
    //                 // 4.642785257361385
    //                 // 7.092341618432331
    //                 // 9.739697410905674

    //                     entry(
    //                         Double.valueOf(1.41567635288239), 
    //                         Double.valueOf(1.2)
    //                     ),
    //                     entry(
    //                         Double.valueOf(2.00764727190795), 
    //                         Double.valueOf(1.2)
    //                     ),
    //                     entry(
    //                         Double.valueOf(2.81173626842406), 
    //                         Double.valueOf(1 + 16.0/60)
    //                     ),
    //                     entry(
    //                         Double.valueOf(3.4956144644965), 
    //                         Double.valueOf(1 + 18.0/60)
    //                     ),
    //                     entry(
    //                         Double.valueOf(4.25871165123726), 
    //                         Double.valueOf(1 + 18.0/60)
    //                     ),
    //                     entry(
    //                         Double.valueOf(4.84412724500999), 
    //                         Double.valueOf(1 + 23.0/60)
    //                     ),
    //                     entry(
    //                         Double.valueOf(5.37548577935445), 
    //                         Double.valueOf(1.4)
    //                     ),
    //                     entry(
    //                         Double.valueOf(6.22187555861643), 
    //                         Double.valueOf(1 + 26.0/60)
    //                     )
    //                 ));

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
