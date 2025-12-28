package org.firstinspires.ftc.teamcode.Robot;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Robot.Subsystems.IntakeSubsystem;

import org.firstinspires.ftc.teamcode.Robot.Commands.CollectCommand;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.TurresSubsystem;


public class RobotContainer {
    private IntakeSubsystem m_intake;
    private TurresSubsystem m_turres;
    private ShooterSubsystem m_shooter;

    private GamepadEx m_driverGamepad;

    private VoltageSensor voltageSensor;
    private double m_voltageSensorValue;

    public enum RobotMode
    {
        AUTO_BLUE,
        AUTO_RED,
        TELEOP_RED,
        TELEOP_BLUE
    }

    RobotContainer(RobotMode robotMode, HardwareMap hmap, IntakeSubsystem intake)
    {
        voltageSensor = hmap.get(VoltageSensor.class,"Control Hub");
        m_intake = intake;
        if (robotMode == RobotMode.TELEOP_RED)
        {
            ConfigureREDBindings();
        }
        else if (robotMode == RobotMode.TELEOP_BLUE)
        {
            ConfigureBLUEBindings();
        }
    }

    public void SetSubsystems(IntakeSubsystem intake, TurresSubsystem turres, ShooterSubsystem shooter)
    {
        m_intake = intake;
        m_turres = turres;
        m_shooter = shooter;
    }

    public void ActualiseVoltageSensorValue()
    {
        m_voltageSensorValue = voltageSensor.getVoltage();
    }

    public double GetVoltageSensorValue()
    {
        return m_voltageSensorValue;
    }

    private void ConfigureREDBindings()
    {
        Gamepad driverGamepadInit = new Gamepad();
        driverGamepadInit.setGamepadId(1);
        m_driverGamepad = new GamepadEx(driverGamepadInit);

        m_driverGamepad.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
                new CollectCommand(m_intake)
        );
    }

    private void ConfigureBLUEBindings()
    {
        Gamepad driverGamepadInit = new Gamepad();
        driverGamepadInit.setGamepadId(1);
        m_driverGamepad = new GamepadEx(driverGamepadInit);

        m_driverGamepad.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
                new CollectCommand(m_intake)
        );
    }
}
