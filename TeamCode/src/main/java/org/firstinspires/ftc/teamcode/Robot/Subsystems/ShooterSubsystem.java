package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;

import static org.firstinspires.ftc.teamcode.Robot.Constants.shooterMaxDistanceOfShoot;
import static org.firstinspires.ftc.teamcode.Robot.Constants.shooterNominalVoltage;
import static org.firstinspires.ftc.teamcode.Robot.Constants.shooterVelocityTolerance;
import static org.firstinspires.ftc.teamcode.Robot.Constants.ShooterKP;
import static org.firstinspires.ftc.teamcode.Robot.Constants.ShooterKI;
import static org.firstinspires.ftc.teamcode.Robot.Constants.ShooterKD;
import static org.firstinspires.ftc.teamcode.Robot.Constants.shooterShootVelocity;
import static org.firstinspires.ftc.teamcode.Robot.Constants.shooterEjectVelocity;
import static org.firstinspires.ftc.teamcode.Robot.Constants.cameraDistanceScaleCoef;

import org.firstinspires.ftc.teamcode.Robot.RobotContainer;
import org.firstinspires.ftc.teamcode.lib.PidRBL;
import org.firstinspires.ftc.teamcode.lib.Utils;
import org.firstinspires.ftc.teamcode.lib.Dashboard;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ShooterSubsystem extends SubsystemBase{
    public enum WantedState
    {
        STAND_BY,
        EJECT_BALL,
        SHOOT
    }

    public enum SystemState
    {
        IDLE,
        PREPARING_TO_EJECT,
        PREPARING_TO_SHOOT,
        READY_TO_EJECT,
        READY_TO_SHOOT
    }
    private final DcMotorEx m_rightMotor;
    private final DcMotor m_leftMotor;
    private Servo m_shooterServo;

    private PidRBL m_motorsPIDController;
    private double m_currentMotorsVelocity = 0.0;

    private WantedState m_wantedState = WantedState.STAND_BY;
    private SystemState m_systemState = SystemState.IDLE;

    private double m_servoTargetPos, m_currentServoTargetPos;

    private final RobotContainer robot;

    ShooterSubsystem (HardwareMap hmap, RobotContainer robot)
    {
        m_rightMotor = hmap.get(DcMotorEx.class, "ShooterRightMotorAndShooterEncoder");
        m_leftMotor = hmap.get(DcMotorEx.class, "ShooterLeftMotor");

        m_rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        m_rightMotor.setDirection(DcMotorSimple.Direction.FORWARD); //TUNEME
        m_rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m_rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        m_leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        m_leftMotor.setDirection(DcMotorSimple.Direction.REVERSE); //TUNEME
        m_leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m_leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        m_motorsPIDController = new PidRBL(ShooterKP, ShooterKI, ShooterKD);
        m_motorsPIDController.SetTolerance(shooterVelocityTolerance);

        this.robot = robot;
    }

    @Override
    public void periodic()
    {
        UpdateInputs();

        RunStateMachine();

        switch (m_systemState)
        {
            case IDLE:
                SetMotorPower(0.0);
                break;

            case READY_TO_EJECT:
            case READY_TO_SHOOT:
                break;

            case PREPARING_TO_EJECT:
                SetMotorPower(m_motorsPIDController.Calculate(shooterEjectVelocity,m_currentMotorsVelocity));
                m_shooterServo.setPosition(m_servoTargetPos);
                break;

            case PREPARING_TO_SHOOT:
                SetMotorPower(m_motorsPIDController.Calculate(shooterShootVelocity,m_currentMotorsVelocity));
                m_shooterServo.setPosition(m_servoTargetPos);
                break;

            default:
                Dashboard.Telemetry_with_Text("Shooter", "unknown system state used");
                SetMotorPower(0.0);
                break;
        }
    }

    private void SetMotorPower(double motorPower)
    {
        motorPower = GetVoltageCompensed(motorPower);
        if (motorPower > 1.0)
        {
            motorPower = 1.0;
        }
        else if (motorPower < -1.0)
        {
            motorPower = -1.0;
        }

        m_rightMotor.setPower(motorPower);
        m_leftMotor.setPower(motorPower);
    }

    private double GetVoltageCompensed(double value)
    {
        return robot.GetVoltageSensorValue()*value/shooterNominalVoltage;
    }

    private void UpdateInputs()
    {
        m_currentMotorsVelocity = m_rightMotor.getVelocity();
        m_currentServoTargetPos = m_shooterServo.getPosition();
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
                if (m_systemState != SystemState.READY_TO_SHOOT)
                {
                    m_systemState = SystemState.PREPARING_TO_SHOOT;
                }
                break;

            case EJECT_BALL:
                if (m_systemState != SystemState.READY_TO_EJECT)
                {
                    m_systemState = SystemState.PREPARING_TO_EJECT;
                }
                break;

            default:
                Dashboard.Telemetry_with_Text("Shooter", "can't run state machine with an unknown wanted state");
                break;
        }

        switch (m_systemState)
        {
            case IDLE:
                break;

            case PREPARING_TO_EJECT:
                if (Utils.IsInRange(m_currentMotorsVelocity, shooterEjectVelocity, shooterVelocityTolerance))
                {
                    m_systemState = SystemState.READY_TO_EJECT;
                }
                break;

            case PREPARING_TO_SHOOT:
                UpdateServoTargetPos();
                if (Utils.IsInRange(m_currentMotorsVelocity, shooterShootVelocity, shooterVelocityTolerance) && m_currentServoTargetPos == m_servoTargetPos
                )
                {
                    m_systemState = SystemState.READY_TO_SHOOT;
                }
                break;

            case READY_TO_EJECT:
            case READY_TO_SHOOT:
                UpdateServoTargetPos();
                break;

            default:
                Dashboard.Telemetry_with_Text("Shooter", "can't run state machine with an unknown system state");
                break;
        }
    }

    private void UpdateServoTargetPos()
    {
        double targetArea = robot.GetCameraTargetArea();
        if (targetArea > 0.0)
        {
            //TODO : corrected target area & recheck this method
            double targetDistance = Math.sqrt(targetArea)/cameraDistanceScaleCoef;
            m_servoTargetPos = targetDistance/shooterMaxDistanceOfShoot;
        }
    }
}
