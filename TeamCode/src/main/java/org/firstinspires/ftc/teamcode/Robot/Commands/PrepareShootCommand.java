package org.firstinspires.ftc.teamcode.Robot.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Robot.Subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.TurresSubsystem;

public class PrepareShootCommand extends CommandBase {
    private final TurresSubsystem m_turresSubsystem;
    private final ShooterSubsystem m_shooterSubsystem;

    public PrepareShootCommand(TurresSubsystem turres, ShooterSubsystem shooter)
    {
        m_turresSubsystem = turres;
        m_shooterSubsystem = shooter;
        addRequirements(turres, shooter);
    }

    @Override
    public void initialize()
    {
        m_shooterSubsystem.SetWantedState(ShooterSubsystem.WantedState.SHOOT);
        m_turresSubsystem.SetWantedState(TurresSubsystem.WantedState.ALINE_WITH_TARGET);
    }

    @Override
    public boolean isFinished()
    {
        return true;
    }
}
