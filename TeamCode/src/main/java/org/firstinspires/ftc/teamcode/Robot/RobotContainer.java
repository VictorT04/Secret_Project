package org.firstinspires.ftc.teamcode.Robot;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Robot.Subsystems.IntakeSubsystem;

import org.firstinspires.ftc.teamcode.Robot.Commands.CollectCommand;


public class RobotContainer {
    private IntakeSubsystem m_intake;

    private GamepadEx m_driverGamepad;

    public enum RobotMode
    {
        AUTO_BLUE,
        AUTO_RED,
        TELEOP_RED,
        TELEOP_BLUE
    }

    RobotContainer(RobotMode robotMode, IntakeSubsystem intake)
    {
        m_intake = intake;
        if (robotMode == RobotMode.TELEOP_BLUE || robotMode == RobotMode.TELEOP_RED)
        {
            ConfigureBindings();
        }
    }

    private void ConfigureBindings()
    {
        Gamepad driverGamepadInit = new Gamepad();
        driverGamepadInit.setGamepadId(1);
        m_driverGamepad = new GamepadEx(driverGamepadInit);

        m_driverGamepad.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
                new CollectCommand(m_intake)
        );
    }
}
