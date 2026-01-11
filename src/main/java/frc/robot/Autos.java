package frc.robot;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class Autos {
    private final AutoFactory m_factory;

    public Autos(AutoFactory factory) {
        m_factory = factory;
    }

    public Command simplePathAutoCommand() {
        return Commands.sequence(
            m_factory.resetOdometry("SimplePath"),
            m_factory.trajectoryCmd("SimplePath")
        );
    }

    public Command testAutoPaths() {
        return Commands.sequence(
            m_factory.resetOdometry("TestPath2026"),
            m_factory.trajectoryCmd("TestPath2026")
        );
    }
}