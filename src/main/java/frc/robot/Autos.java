package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class Autos {
    private final AutoFactory m_factory;

    public Autos(AutoFactory factory) {
        m_factory = factory;
    }

    public Command testAutoPaths() {
        return Commands.sequence(
            Commands.print("TESTTESTTEST"),
            m_factory.resetOdometry("TestPath2026"),
            m_factory.trajectoryCmd("TestPath2026"),
            Commands.print("TESTTESTTE2w23232323ST")
        );
    }

    public AutoRoutine simplePathAuto() {
        final AutoRoutine routine = m_factory.newRoutine("SimplePath Auto");
        final AutoTrajectory simplePath = routine.trajectory("TestPath2026");

        routine.active().onTrue(
            simplePath.resetOdometry()
                .andThen(simplePath.cmd())
        );
        return routine;
    }
}