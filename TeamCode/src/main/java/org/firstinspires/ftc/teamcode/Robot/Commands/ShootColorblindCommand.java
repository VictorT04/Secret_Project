package org.firstinspires.ftc.teamcode.Robot.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Robot.Subsystems.FeederSubsystem;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.TurresSubsystem;

public class ShootColorblindCommand extends CommandBase {
    private final FeederSubsystem m_feederSubsystem;
    private final TurresSubsystem m_turresSubsystem;
    private final ShooterSubsystem m_shooterSubsystem;

    public ShootColorblindCommand(FeederSubsystem feeder, TurresSubsystem turres, ShooterSubsystem shooter)
    {
        m_feederSubsystem = feeder;
        m_turresSubsystem = turres;
        m_shooterSubsystem = shooter;
        addRequirements(feeder,turres,shooter);
    }

    @Override
    public void initialize()
    {
        if (m_turresSubsystem.GetSystemState() == TurresSubsystem.SystemState.ALINED_TO_TARGET &&
                m_shooterSubsystem.GetSystemState() == ShooterSubsystem.SystemState.READY_TO_SHOOT)
        {
            m_feederSubsystem.SetWantedState(FeederSubsystem.WantedState.FEED_COLORBLIND);
        }
    }

    @Override
    public boolean isFinished()
    {
        return true;
    }
}
