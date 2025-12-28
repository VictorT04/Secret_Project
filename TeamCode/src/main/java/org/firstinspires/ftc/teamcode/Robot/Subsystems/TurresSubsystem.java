package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Robot.RobotContainer;
import org.firstinspires.ftc.teamcode.lib.PidRBL;

import static org.firstinspires.ftc.teamcode.Robot.Constants.turresKP;
import static org.firstinspires.ftc.teamcode.Robot.Constants.turresKI;
import static org.firstinspires.ftc.teamcode.Robot.Constants.turresKD;
import static org.firstinspires.ftc.teamcode.Robot.Constants.turresNominalVoltage;
import static org.firstinspires.ftc.teamcode.Robot.Constants.turresOrientationTolerance;

public class TurresSubsystem {
    private final DcMotor m_turresMotor;

    private PidRBL m_turresPIDController;

    private WantedState m_wantedState = WantedState.STAND_BY;
    private SystemState m_systemState = SystemState.IDLE;

    private double m_targetPosition;

    private double m_turresHeading;

    private RobotContainer robot;

    public enum WantedState
    {
        STAND_BY,
        IDENTIFY_OBELISK,
        ALINE_WITH_TARGET,
        RETURN_HOME
    }

    public enum SystemState
    {
        IDLE,
        SEARCHING_OBELISK,
        SEARCHING_TARGET,
        ALINEYING_TO_TARGET,
        ALINED_TO_TARGET,
        RETURNING_HOME,
        AT_HOME
    }

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

    public void RunStateMachine()
    {
        switch (m_wantedState)
        {
            case STAND_BY:
            case RETURN_HOME:
                if (m_systemState != SystemState.AT_HOME)
                    m_systemState = SystemState.RETURNING_HOME;
                break;

            case IDENTIFY_OBELISK:
                if (m_systemState != SystemState.SEARCHING_OBELISK)
                    m_systemState = SystemState.SEARCHING_OBELISK;
                break;

            case ALINE_WITH_TARGET:
                if (m_systemState != SystemState.ALINED_TO_TARGET && m_systemState != SystemState.ALINEYING_TO_TARGET)
                    m_systemState = SystemState.SEARCHING_TARGET;
                break;

            default:
                //TODO
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
                //TODO
                break;

            case ALINEYING_TO_TARGET:
                if (m_turresHeading <= m_targetPosition + turresOrientationTolerance && m_turresHeading >= m_targetPosition-turresOrientationTolerance)
                    m_systemState = SystemState.ALINED_TO_TARGET;
                break;

            case SEARCHING_OBELISK:
                break;

            case ALINED_TO_TARGET:
                break;

            case AT_HOME:
                break;

        }
    }
}
