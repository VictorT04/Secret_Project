package org.firstinspires.ftc.teamcode.Robot.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Robot.Subsystems.FeederSubsystem;
import  org.firstinspires.ftc.teamcode.Robot.Subsystems.IntakeSubsystem;

public class CollectCommand extends CommandBase{
    private final IntakeSubsystem m_intakeSubsystem;

    private final FeederSubsystem m_feederSubsystem;

    public CollectCommand (IntakeSubsystem intake, FeederSubsystem feeder)
    {
        m_intakeSubsystem = intake;
        m_feederSubsystem = feeder;
        addRequirements(intake, feeder);
    }

    @Override
    public void initialize()
    {
        if (!m_feederSubsystem.IsFull())
        {
            m_intakeSubsystem.SetWantedState(IntakeSubsystem.WantedState.COLLECT);
            m_feederSubsystem.SetWantedState(FeederSubsystem.WantedState.IDENTIFY_COLLECTED_BALL);
        }
    }

    @Override
    public boolean isFinished()
    {
        return true;
    }
}
