package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;

import static org.firstinspires.ftc.teamcode.Robot.Constants.shooterVelocityTolerance;
import static org.firstinspires.ftc.teamcode.Robot.Constants.topShooterKP;
import static org.firstinspires.ftc.teamcode.Robot.Constants.topShooterKI;
import static org.firstinspires.ftc.teamcode.Robot.Constants.topShooterKD;
import static org.firstinspires.ftc.teamcode.Robot.Constants.bottomShooterKP;
import static org.firstinspires.ftc.teamcode.Robot.Constants.bottomShooterKI;
import static org.firstinspires.ftc.teamcode.Robot.Constants.bottomShooterKD;

import org.firstinspires.ftc.teamcode.lib.PidRBL;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ShooterSubsystem extends SubsystemBase{
    private DcMotorEx m_topMotor;
    private DcMotorEx m_bottomMotor;
    private Servo m_shooterServo;

    private PidRBL m_topMotorPIDController, m_bottomMotorPIDController;

    private double m_targetVelocity = 0.0;

    private WantedState m_wantedState = WantedState.STAND_BY;
    private SystemState m_systemState = SystemState.IDLE;

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

    ShooterSubsystem (HardwareMap hmap)
    {
        m_topMotor = hmap.get(DcMotorEx.class, "ShooterTopMotor");
        m_bottomMotor = hmap.get(DcMotorEx.class, "ShooterBottomMotor");

        m_topMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        m_topMotor.setDirection(DcMotorSimple.Direction.FORWARD); //TUNEME
        m_topMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m_topMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        m_bottomMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        m_bottomMotor.setDirection(DcMotorSimple.Direction.FORWARD); //TUNEME
        m_bottomMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m_bottomMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        m_bottomMotorPIDController = new PidRBL(topShooterKP, topShooterKI, topShooterKD);
        m_bottomMotorPIDController.SetTolerance(shooterVelocityTolerance);

        m_topMotorPIDController = new PidRBL(bottomShooterKP, bottomShooterKI, bottomShooterKD);
        m_topMotorPIDController.SetTolerance(shooterVelocityTolerance);
    }

    @Override
    public void periodic()
    {
        RunStateMachine();

        switch (m_systemState)
        {
            case AT_SHOOT_VELOCITY:
            case AT_EJECT_VELOCITY:
                break;

            case RAMPING_TO_SHOOT:
            case RAMPING_TO_EJECT:
                m_bottomMotor.setPower(m_bottomMotorPIDController.Calculate(m_targetVelocity,m_bottomMotor.getVelocity()));
                m_topMotor.setPower(m_topMotorPIDController.Calculate(m_targetVelocity,m_topMotor.getVelocity()));
        }
    }

    public void RunStateMachine()
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
                if (m_topMotor.getVelocity() >= m_targetVelocity-shooterVelocityTolerance &&
                    m_topMotor.getVelocity() <= m_targetVelocity+shooterVelocityTolerance &&
                    m_bottomMotor.getVelocity() >= m_targetVelocity-shooterVelocityTolerance &&
                    m_bottomMotor.getVelocity() <= m_targetVelocity+shooterVelocityTolerance)
                {
                    m_systemState = SystemState.AT_EJECT_VELOCITY;
                }
                else
                {
                    //TODO : set m_targetVelocity with target distance
                }
                break;

            case RAMPING_TO_SHOOT:
                if (m_topMotor.getVelocity() >= m_targetVelocity-shooterVelocityTolerance &&
                        m_topMotor.getVelocity() <= m_targetVelocity+shooterVelocityTolerance &&
                        m_bottomMotor.getVelocity() >= m_targetVelocity-shooterVelocityTolerance &&
                        m_bottomMotor.getVelocity() <= m_targetVelocity+shooterVelocityTolerance)
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
                //TODO
                break;

            default:
                break;
        }
    }
}
