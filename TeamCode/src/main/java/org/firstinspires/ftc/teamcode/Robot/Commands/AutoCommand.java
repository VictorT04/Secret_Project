package org.firstinspires.ftc.teamcode.Robot.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Robot.Subsystems.FeederSubsystem;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.TurresSubsystem;

public class AutoCommand extends CommandBase {
    /*private final FeederSubsystem m_feederSubsystem;
    private final TurresSubsystem m_turresSubsystem;
    private final ShooterSubsystem m_shooterSubsystem;
    private final IntakeSubsystem m_intakeSubsystem;
    private final MecanumDrivetrain m_drivetrain;*/

    public AutoCommand(FeederSubsystem feeder, TurresSubsystem turres, ShooterSubsystem shooter, IntakeSubsystem intake, MecanumDrivetrain drivetrain,
                        String robotTrajectory)
    {
        /*m_feederSubsystem = feeder;
        m_turresSubsystem = turres;
        m_shooterSubsystem = shooter;
        m_intakeSubsystem = intake;
        m_drivetrain = drivetrain;*/
    }

    @Override
    public void initialize()
    {

    }

    @Override
    public void execute()
    {

    }

    @Override
    public boolean isFinished()
    {
        return true;
    }
}
