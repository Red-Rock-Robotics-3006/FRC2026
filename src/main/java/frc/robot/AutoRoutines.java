package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class AutoRoutines {
    private final AutoFactory m_factory;

    public AutoRoutines(AutoFactory factory) {
        m_factory = factory;
    }

    public AutoRoutine simplePathAuto() {
        final AutoRoutine routine = m_factory.newRoutine("SimplePath Auto");
        final AutoTrajectory simplePath = routine.trajectory("SimplePath");

        routine.active().onTrue(
            simplePath.resetOdometry()
                .andThen(simplePath.cmd())
        );
        return routine;
    }

    public Command simplePathAutoCommand() {
        return Commands.sequence(
            Commands.print("MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM"),
            m_factory.resetOdometry("SimplePath"),
            m_factory.trajectoryCmd("SimplePath"),
            Commands.print("M2222222222222222222MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM")
        );
    }

    public Command testAutoPaths() {
        return Commands.sequence(
            Commands.print("TESTTESTTEST"),
            m_factory.resetOdometry("TestPath2026"),
            m_factory.trajectoryCmd("TestPath2026"),
            Commands.print("TESTTESTTE2w23232323ST")
        );
    }
}