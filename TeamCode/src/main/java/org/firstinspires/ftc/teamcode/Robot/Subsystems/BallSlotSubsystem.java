package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import android.graphics.Color;
import android.os.Build;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.teamcode.lib.Utils;

import static org.firstinspires.ftc.teamcode.Robot.Constants.GreenBallRedValue;
import static org.firstinspires.ftc.teamcode.Robot.Constants.GreenBallGreenValue;
import static org.firstinspires.ftc.teamcode.Robot.Constants.GreenBallBlueValue;
import static org.firstinspires.ftc.teamcode.Robot.Constants.PurpleBallRedValue;
import static org.firstinspires.ftc.teamcode.Robot.Constants.PurpleBallGreenValue;
import static org.firstinspires.ftc.teamcode.Robot.Constants.PurpleBallBlueValue;
import static org.firstinspires.ftc.teamcode.Robot.Constants.ColorSensorTolerance;
import static org.firstinspires.ftc.teamcode.Robot.Constants.SlotServoFeedingPos;
import static org.firstinspires.ftc.teamcode.Robot.Constants.SlotServoHomePos;

import androidx.annotation.RequiresApi;

public class BallSlotSubsystem {
    private final Servo m_slotServo;
    private final ColorSensor m_slotColorSensor;
    private final DistanceSensor m_slotDistanceSensor;

    private SlotState m_slotState;
    private Color m_colorSensorResult;
    private double m_distanceSensorValue; //in mm
    private double m_servoPos;
    public enum SlotState
    {
        EMPTY,
        GREEN,
        PURPLE,
        UNKNOWN
    }

    BallSlotSubsystem(HardwareMap hmap, String slotName)
    {
        m_slotServo = hmap.get(Servo.class,slotName + "SlotServo");
        m_slotColorSensor = hmap.get(ColorSensor.class, slotName + "SlotColorSensor");
        m_slotDistanceSensor = hmap.get(DistanceSensor.class, slotName + "SlotDistanceSensor");
    }

    public SlotState getSlotState()
    {
        return m_slotState;
    }

    @RequiresApi(api = Build.VERSION_CODES.O)
    public void UpdateInputs()
    {
        m_colorSensorResult = Color.valueOf(m_slotColorSensor.argb());
        m_distanceSensorValue = m_slotDistanceSensor.getDistance(DistanceUnit.MM);
        m_servoPos = m_slotServo.getPosition();
    }

    @RequiresApi(api = Build.VERSION_CODES.O)
    public boolean IsThereANewBall()
    {
        SlotState previousSlotState = m_slotState;
        IdentifyBallColor();
        return previousSlotState == m_slotState && m_slotState != SlotState.EMPTY;
    }

    @RequiresApi(api = Build.VERSION_CODES.O)
    public void IdentifyBallColor()
    {
        if (Utils.IsInRange(m_colorSensorResult.red(), PurpleBallRedValue,ColorSensorTolerance) && Utils.IsInRange(m_colorSensorResult.green(), PurpleBallGreenValue,ColorSensorTolerance) && Utils.IsInRange(m_colorSensorResult.blue(), PurpleBallBlueValue,ColorSensorTolerance))
        {
            m_slotState = SlotState.PURPLE;
        }
        else if (Utils.IsInRange(m_colorSensorResult.red(), GreenBallRedValue,ColorSensorTolerance) && Utils.IsInRange(m_colorSensorResult.green(), GreenBallGreenValue,ColorSensorTolerance) && Utils.IsInRange(m_colorSensorResult.blue(), GreenBallBlueValue,ColorSensorTolerance))
        {
            m_slotState = SlotState.GREEN;
        }
        else if (m_distanceSensorValue > 70.0)
        {
            m_slotState = SlotState.EMPTY;
        }
        else
        {
            m_slotState = SlotState.UNKNOWN;
        }

    }

    public void FeedBall()
    {
        m_slotServo.setPosition(SlotServoFeedingPos);
        m_slotState = SlotState.EMPTY;
    }

    public void ReturnServoToHome()
    {
        m_slotServo.setPosition(SlotServoHomePos);
    }

    public void SetSlotState()
    {
        //TODO
    }

    public double GetServoPos()
    {
        return m_servoPos;
    }


}
