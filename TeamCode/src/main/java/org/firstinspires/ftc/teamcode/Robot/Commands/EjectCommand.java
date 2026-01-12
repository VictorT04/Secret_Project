package org.firstinspires.ftc.teamcode.Robot.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Robot.Subsystems.FeederSubsystem;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ShooterSubsystem;
public class EjectCommand extends CommandBase {
    private final FeederSubsystem m_feederSubsystem;
    private final ShooterSubsystem m_shooterSubsystem;

    public EjectCommand(FeederSubsystem feeder, ShooterSubsystem shooter)
    {
        m_feederSubsystem = feeder;
        m_shooterSubsystem = shooter;
        addRequirements(feeder,shooter);
    }

    @Override
    public void initialize()
    {
        if (m_shooterSubsystem.GetSystemState() == ShooterSubsystem.SystemState.READY_TO_EJECT)
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
