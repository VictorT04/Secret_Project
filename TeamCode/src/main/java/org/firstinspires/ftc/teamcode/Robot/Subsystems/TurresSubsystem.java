package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Robot.RobotContainer;
import org.firstinspires.ftc.teamcode.lib.Dashboard;
import org.firstinspires.ftc.teamcode.lib.PidRBL;
import org.firstinspires.ftc.teamcode.lib.Utils;

import static org.firstinspires.ftc.teamcode.Robot.Constants.turresKP;
import static org.firstinspires.ftc.teamcode.Robot.Constants.turresKI;
import static org.firstinspires.ftc.teamcode.Robot.Constants.turresKD;
import static org.firstinspires.ftc.teamcode.Robot.Constants.turresNominalVoltage;
import static org.firstinspires.ftc.teamcode.Robot.Constants.turresOrientationTolerance;

public class TurresSubsystem extends SubsystemBase {

    public enum WantedState
    {
        STAND_BY,
        ALINE_WITH_TARGET,
        RETURN_HOME
    }

    public enum SystemState
    {
        IDLE,
        SEARCHING_TARGET,
        ALINEYING_TO_TARGET,
        ALINED_TO_TARGET,
        RETURNING_HOME,
        AT_HOME
    }
    private final DcMotor m_turresMotor;

    private PidRBL m_turresPIDController;

    private WantedState m_wantedState = WantedState.STAND_BY;
    private SystemState m_systemState = SystemState.IDLE;

    private double m_targetPosition;

    private double m_turresHeading; //in degrees

    private RobotContainer robot;

    private double m_targetY;

    public TurresSubsystem(HardwareMap hmap, RobotContainer robot)
    {
        m_turresMotor = hmap.get(DcMotor.class,"turresMotor");

        m_turresMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m_turresMotor.setDirection(DcMotorSimple.Direction.FORWARD); //TUNEME
        m_turresMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m_turresMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        m_turresPIDController = new PidRBL (turresKP, turresKI, turresKD);
        m_turresPIDController.SetTolerance(turresOrientationTolerance);

        this.robot = robot;
    }

    public void SetWantedState(WantedState wantedState)
    {
        m_wantedState = wantedState;
    }

    public SystemState GetSystemState()
    {
        return m_systemState;
    }

    private void UpdateInputs()
    {
        m_turresHeading = (m_turresMotor.getCurrentPosition() % 8192.0) / 8192.0/360;
    }

    private void SetMotorPower(double power)
    {
        power = robot.GetVoltageSensorValue()*power/turresNominalVoltage;
        if (power > 1.0)
        {
            power = 1.0;
        }
        else if (power < -1.0)
        {
            power = -1.0;
        }
        m_turresMotor.setPower(power);
    }

    @Override
    public void periodic()
    {
        UpdateInputs();

        RunStateMachine();

        switch(m_systemState)
        {
            case SEARCHING_TARGET:
            case ALINED_TO_TARGET:
            case AT_HOME:
            case IDLE:
                SetMotorPower(0.0);
                break;

            case ALINEYING_TO_TARGET:
                SetMotorPower(m_turresPIDController.Calculate(m_targetPosition, m_turresHeading));
                break;

            case RETURNING_HOME:
                SetMotorPower(m_turresPIDController.Calculate(0.0, m_turresHeading));
                break;

            default:
                Dashboard.Telemetry_with_Text("Turres", "unknown system state used");
        }
    }

    public void RunStateMachine()
    {
        switch (m_wantedState)
        {
            case STAND_BY:
            case RETURN_HOME:
                if (m_systemState != SystemState.AT_HOME)
                    m_systemState = SystemState.RETURNING_HOME;
                break;

            case ALINE_WITH_TARGET:
                if (m_systemState != SystemState.ALINED_TO_TARGET && m_systemState != SystemState.ALINEYING_TO_TARGET)
                    m_systemState = SystemState.SEARCHING_TARGET;
                break;

            default:
                Dashboard.Telemetry_with_Text("Turres", "can't run state machine with an unknown wanted state");
                break;
        }

        switch (m_systemState)
        {
            case IDLE:
                m_systemState = SystemState.RETURNING_HOME;
                break;

            case RETURNING_HOME:
                if (m_turresHeading <= turresOrientationTolerance && m_turresHeading >= -turresOrientationTolerance)
                    m_systemState = SystemState.AT_HOME;
                break;

            case SEARCHING_TARGET:
                m_targetY = robot.GetCameraTargetY();
                if (m_targetY != 90.0)
                {
                    m_targetPosition = m_turresHeading-m_targetY;
                    m_systemState = SystemState.ALINEYING_TO_TARGET;
                }
                break;

            case ALINEYING_TO_TARGET:
                m_targetY = robot.GetCameraTargetY();
                if (m_targetY != 90.0)
                {
                    m_targetPosition = m_turresHeading-m_targetY;
                }
                else
                {
                    m_systemState = SystemState.SEARCHING_TARGET;
                }
                if (Utils.IsInRange(m_turresHeading, m_targetPosition,turresOrientationTolerance))
                    m_systemState = SystemState.ALINED_TO_TARGET;
                break;

            case ALINED_TO_TARGET:
                if (!Utils.IsInRange(m_turresHeading, m_targetPosition,turresOrientationTolerance))
                {
                    m_targetY = robot.GetCameraTargetY();
                    if (m_targetY != 90.0)
                    {
                        m_targetPosition = m_turresHeading-m_targetY;
                    }
                    else
                    {
                        m_systemState = SystemState.SEARCHING_TARGET;
                    }
                }
                break;

            case AT_HOME:
                break;

            default:
                Dashboard.Telemetry_with_Text("Turres", "can't run state machine with an unknown system state");
                break;
        }
    }
}
