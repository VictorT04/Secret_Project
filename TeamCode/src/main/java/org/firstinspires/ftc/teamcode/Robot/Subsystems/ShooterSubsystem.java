package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;

import static org.firstinspires.ftc.teamcode.Robot.Constants.shooterNominalVoltage;
import static org.firstinspires.ftc.teamcode.Robot.Constants.shooterVelocityTolerance;
import static org.firstinspires.ftc.teamcode.Robot.Constants.topShooterKP;
import static org.firstinspires.ftc.teamcode.Robot.Constants.topShooterKI;
import static org.firstinspires.ftc.teamcode.Robot.Constants.topShooterKD;
import static org.firstinspires.ftc.teamcode.Robot.Constants.bottomShooterKP;
import static org.firstinspires.ftc.teamcode.Robot.Constants.bottomShooterKI;
import static org.firstinspires.ftc.teamcode.Robot.Constants.bottomShooterKD;

import org.firstinspires.ftc.teamcode.Robot.RobotContainer;
import org.firstinspires.ftc.teamcode.lib.PidRBL;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ShooterSubsystem extends SubsystemBase{
    private final DcMotorEx m_rightMotor;
    private final DcMotorEx m_leftMotor;
    private Servo m_shooterServo;

    private PidRBL m_topMotorPIDController, m_bottomMotorPIDController;

    private double m_targetVelocity = 0.0;

    private double m_currentMotorsVelocity = 0.0;

    private WantedState m_wantedState = WantedState.STAND_BY;
    private SystemState m_systemState = SystemState.IDLE;

    private final RobotContainer robot;

    public enum WantedState
    {
        STAND_BY,
        EJECT_BALL,
        SHOOT
    }

    public enum SystemState
    {
        IDLE,
        RAMPING_TO_EJECT,
        RAMPING_TO_SHOOT,
        AT_EJECT_VELOCITY,
        AT_SHOOT_VELOCITY
    }

    ShooterSubsystem (HardwareMap hmap, RobotContainer robot)
    {
        m_rightMotor = hmap.get(DcMotorEx.class, "ShooterRightMotor");
        m_leftMotor = hmap.get(DcMotorEx.class, "ShooterLeftMotor");

        m_rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        m_rightMotor.setDirection(DcMotorSimple.Direction.FORWARD); //TUNEME
        m_rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m_rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        m_leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        m_leftMotor.setDirection(DcMotorSimple.Direction.FORWARD); //TUNEME
        m_leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m_leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        m_bottomMotorPIDController = new PidRBL(topShooterKP, topShooterKI, topShooterKD);
        m_bottomMotorPIDController.SetTolerance(shooterVelocityTolerance);

        m_topMotorPIDController = new PidRBL(bottomShooterKP, bottomShooterKI, bottomShooterKD);
        m_topMotorPIDController.SetTolerance(shooterVelocityTolerance);

        this.robot = robot;
    }

    @Override
    public void periodic()
    {
        UpdateInputs();

        RunStateMachine();

        switch (m_systemState)
        {
            case AT_SHOOT_VELOCITY:
            case AT_EJECT_VELOCITY:
                break;

            case RAMPING_TO_SHOOT:
            case RAMPING_TO_EJECT:
                SetMotorPower(m_bottomMotorPIDController.Calculate(m_targetVelocity,m_currentMotorsVelocity),
                              m_topMotorPIDController.Calculate(m_targetVelocity,m_currentMotorsVelocity));
        }
    }

    private void SetMotorPower(double topMotorPower, double bottomMotorPower)
    {
        topMotorPower = GetVoltageCompensed(topMotorPower);
        bottomMotorPower = GetVoltageCompensed(bottomMotorPower);
        if (topMotorPower > 1.0)
        {
            topMotorPower = 1.0;
        }
        else if (topMotorPower < -1.0)
        {
            topMotorPower = -1.0;
        }

        if (bottomMotorPower > 1.0)
        {
            bottomMotorPower = 1.0;
        }
        else if (bottomMotorPower < -1.0)
        {
            bottomMotorPower = -1.0;
        }

        m_rightMotor.setPower(bottomMotorPower);
        m_leftMotor.setPower(topMotorPower);
    }

    private double GetVoltageCompensed(double value)
    {
        return robot.GetVoltageSensorValue()*value/shooterNominalVoltage;
    }

    private void UpdateInputs()
    {
        m_currentMotorsVelocity = m_rightMotor.getVelocity();
    }

    private void RunStateMachine()
    {
        switch (m_wantedState)
        {
            case STAND_BY:
                if (m_systemState != SystemState.IDLE)
                {
                    m_systemState = SystemState.IDLE;
                }
                break;

            case SHOOT:
                if (m_systemState != SystemState.AT_SHOOT_VELOCITY)
                {
                    m_systemState = SystemState.RAMPING_TO_SHOOT;
                }
                break;

            case EJECT_BALL:
                if (m_systemState != SystemState.AT_EJECT_VELOCITY)
                {
                    m_systemState = SystemState.RAMPING_TO_EJECT;
                }
                break;

            default:
                break;
        }

        switch (m_systemState)
        {
            case IDLE:
                break;

            case RAMPING_TO_EJECT:
                if (m_currentMotorsVelocity >= m_targetVelocity-shooterVelocityTolerance &&
                    m_currentMotorsVelocity <= m_targetVelocity+shooterVelocityTolerance)
                {
                    m_systemState = SystemState.AT_EJECT_VELOCITY;
                }
                else
                {
                    //TODO : set m_targetVelocity with target distance
                }
                break;

            case RAMPING_TO_SHOOT:
                if (m_currentMotorsVelocity >= m_targetVelocity-shooterVelocityTolerance &&
                    m_currentMotorsVelocity <= m_targetVelocity+shooterVelocityTolerance)
                {
                    m_systemState = SystemState.AT_SHOOT_VELOCITY;
                }
                else
                {
                    //TODO : set m_targetVelocity
                }
                break;

            case AT_EJECT_VELOCITY:
            case AT_SHOOT_VELOCITY:
                break;

            default:
                break;
        }
    }
}
