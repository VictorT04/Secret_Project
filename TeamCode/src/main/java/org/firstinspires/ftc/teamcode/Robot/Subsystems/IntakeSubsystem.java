package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.arcrobotics.ftclib.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    private final DcMotor intakeMotor;

    private WantedState m_wantedState = WantedState.STAND_BY;
    private SystemState m_systemState = SystemState.IDLE;


    public enum WantedState
    {
        STAND_BY,
        COLLECT,
    }

    public enum SystemState
    {
        IDLE,
        INTAKING
    }

    public IntakeSubsystem(HardwareMap hmap)
    {
        intakeMotor = hmap.get(DcMotor.class,"intakeMotor");

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD); //TUNEME
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void SetWantedState(WantedState wantedState)
    {
        m_wantedState = wantedState;
    }

    @Override
    public void periodic()
    {
        switch (m_systemState)
        {
            case IDLE:
                intakeMotor.setPower(0.0);
                break;

            case INTAKING:
                intakeMotor.setPower(1.0);
                break;

            default:
                break;
        }
    }

    private void RunStateMachine()
    {
        switch (m_wantedState)
        {
            case COLLECT:
                if (m_systemState != SystemState.INTAKING)
                    m_systemState = SystemState.INTAKING;
                break;

            case STAND_BY:
                if (m_systemState != SystemState.IDLE)
                    m_systemState = SystemState.IDLE;

            default:
                break;

        }
    }

}
