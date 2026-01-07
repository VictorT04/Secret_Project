package org.firstinspires.ftc.teamcode.Robot;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Robot.Subsystems.Camera;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.IntakeSubsystem;

import org.firstinspires.ftc.teamcode.Robot.Commands.CollectCommand;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.TurresSubsystem;
import org.firstinspires.ftc.teamcode.lib.Dashboard;


public class RobotContainer {

    public enum ObeliskPattern
    {
        PPG, //PURPLE->PURPLE->GREEN, Tag ID : 23
        PGP, //PURPLE->GREEN->PURPLE, Tag ID : 22
        GPP, //GREEN->PURPLE->PURPLE, Tag ID : 21
        UNKNOWN
    }
    private IntakeSubsystem m_intake;
    private TurresSubsystem m_turres;
    private ShooterSubsystem m_shooter;
    private Camera m_camera;

    private GamepadEx m_driverGamepad;

    private final VoltageSensor voltageSensor;
    private double m_voltageSensorValue;

    private ObeliskPattern m_gameObelisk = ObeliskPattern.UNKNOWN;

    public enum RobotMode
    {
        AUTO_BLUE,
        AUTO_RED,
        TELEOP_RED,
        TELEOP_BLUE
    }

    RobotContainer(RobotMode robotMode, HardwareMap hmap)
    {
        voltageSensor = hmap.get(VoltageSensor.class,"Control Hub");
        m_camera = new Camera(hmap, robotMode);

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

    public double GetCameraTargetY()
    {
        return m_camera.GetTargetTy();
    }

    public double GetCameraTargetArea()
    {
        return m_camera.GetTargetArea();
    }
}
