package org.firstinspires.ftc.teamcode.Robot.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import  org.firstinspires.ftc.teamcode.Robot.Subsystems.IntakeSubsystem;

public class CollectCommand extends CommandBase{
    private final IntakeSubsystem m_intakeSubsystem;

    public CollectCommand (IntakeSubsystem intake)
    {
        m_intakeSubsystem = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize()
    {
        m_intakeSubsystem.SetWantedState(IntakeSubsystem.WantedState.COLLECT);
    }

    @Override
    public boolean isFinished()
    {
        return true;
    }
}
