package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Robot.RobotContainer;
import org.firstinspires.ftc.teamcode.lib.Dashboard;

import static org.firstinspires.ftc.teamcode.Robot.Constants.intakeNominalVoltage;

public class IntakeSubsystem extends SubsystemBase {

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
    private final DcMotor m_intakeMotor;

    private WantedState m_wantedState = WantedState.STAND_BY;
    private SystemState m_systemState = SystemState.IDLE;

    private final RobotContainer robot;

    public IntakeSubsystem(HardwareMap hmap, RobotContainer robot)
    {
        m_intakeMotor = hmap.get(DcMotor.class,"intakeMotor");

        m_intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        m_intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD); //TUNEME
        m_intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.robot = robot;
    }

    public void SetWantedState(WantedState wantedState)
    {
        m_wantedState = wantedState;
    }

    private void SetMotorPower(double power)
    {
        power = robot.GetVoltageSensorValue()*power/intakeNominalVoltage;
        if (power > 1.0)
        {
            power = 1.0;
        }
        else if (power < -1.0)
        {
            power = -1.0;
        }
        m_intakeMotor.setPower(power);
    }

    @Override
    public void periodic()
    {
        RunStateMachine();

        switch (m_systemState)
        {
            case IDLE:
                SetMotorPower(0.0);
                break;

            case INTAKING:
                SetMotorPower(1.0);
                break;

            default:
                Dashboard.Telemetry_with_Text("Intake", "unknown system state used");
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
                Dashboard.Telemetry_with_Text("Intake", "can't run state machine with an unknown wanted state");
                break;
        }
    }

}
