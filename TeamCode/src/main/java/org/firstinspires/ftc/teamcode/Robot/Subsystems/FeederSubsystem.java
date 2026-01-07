package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import android.os.Build;

import androidx.annotation.RequiresApi;


import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static org.firstinspires.ftc.teamcode.Robot.Constants.SlotServoFeedingPos;

import org.firstinspires.ftc.teamcode.lib.Dashboard;

public class FeederSubsystem extends SubsystemBase {

    public enum WantedState
    {
        STAND_BY,
        IDENTIFY_COLLECTED_BALL,
        FEED_PURPLE,
        FEED_GREEN,
        FEED_COLORBLIND
    }

    public enum SystemState
    {
        IDLE,
        SEARCHING_NEW_BALL,
        SEARCHING_GREEN,
        SEARCHING_PURPLE,
        FEEDING_GREEN,
        FEEDING_PURPLE
    }

    private final BallSlotSubsystem[] m_feederSlots;

    private WantedState m_wantedState = WantedState.STAND_BY;
    private SystemState m_systemState = SystemState.IDLE;

    private int m_feedingSlotID = -1;

    public FeederSubsystem(HardwareMap hmap)
    {
        m_feederSlots = new BallSlotSubsystem[] {new BallSlotSubsystem(hmap, "front",false), new BallSlotSubsystem(hmap, "middle",false), new BallSlotSubsystem(hmap, "back",true)};
    }

    public void SetWantedState(WantedState wantedState)
    {
        m_wantedState = wantedState;
    }

    public SystemState GetSystemState()
    {
        return m_systemState;
    }

    @RequiresApi(api = Build.VERSION_CODES.O)
    @Override
    public void periodic()
    {
        for (BallSlotSubsystem slot : m_feederSlots)
        {
            slot.UpdateInputs();
            if (slot.getSlotState() == BallSlotSubsystem.SlotState.UNKNOWN)
            {
                slot.IdentifyBallColor();
            }
        }

        RunStateMachine();
    }

    public boolean IsFull()
    {
        for (BallSlotSubsystem slot : m_feederSlots)
        {
            if (slot.getSlotState() == BallSlotSubsystem.SlotState.EMPTY)
            {
                return false;
            }
        }
        return true;
    }

    private boolean IsEmpty()
    {
        for (BallSlotSubsystem slot : m_feederSlots)
        {
            if (slot.getSlotState() != BallSlotSubsystem.SlotState.EMPTY)
            {
                return false;
            }
        }
        return true;
    }

    private int GetNumberOfGreenBall()
    {
        int result = 0;
        for (BallSlotSubsystem slot : m_feederSlots)
        {
            if (slot.getSlotState() != BallSlotSubsystem.SlotState.GREEN)
            {
                result++;
            }
        }
        return result;
    }

    private int GetNumberOfPurpleBall()
    {
        int result = 0;
        for (BallSlotSubsystem slot : m_feederSlots)
        {
            if (slot.getSlotState() != BallSlotSubsystem.SlotState.PURPLE)
            {
                result++;
            }
        }
        return result;
    }

    @RequiresApi(api = Build.VERSION_CODES.O)
    private void RunStateMachine()
    {
        switch (m_wantedState)
        {
            case STAND_BY:
                break;

            case IDENTIFY_COLLECTED_BALL:
                if (!IsFull())
                {
                    if (m_systemState != SystemState.SEARCHING_NEW_BALL)
                    {
                        m_systemState = SystemState.SEARCHING_NEW_BALL;
                    }
                }
                else
                {
                    m_wantedState = WantedState.STAND_BY;
                }
                break;

            case FEED_GREEN:
                if (GetNumberOfGreenBall() > 0)
                {
                    if (m_systemState != SystemState.SEARCHING_GREEN && m_systemState != SystemState.FEEDING_GREEN)
                    {
                        m_systemState = SystemState.SEARCHING_GREEN;
                    }
                }
                else
                {
                    m_wantedState = WantedState.STAND_BY;
                }
                break;

            case FEED_PURPLE:
                if (GetNumberOfPurpleBall() > 0)
                {
                    if (m_systemState != SystemState.SEARCHING_PURPLE && m_systemState != SystemState.FEEDING_PURPLE)
                    {
                        m_systemState = SystemState.SEARCHING_PURPLE;
                    }
                }
                else
                {
                    m_wantedState = WantedState.STAND_BY;
                }
                break;

            case FEED_COLORBLIND:
                int nbGreenBall = GetNumberOfGreenBall(), nbPurpleBall = GetNumberOfPurpleBall();
                if (nbGreenBall > 0 || nbPurpleBall > 0)
                {
                    if (nbPurpleBall >= nbGreenBall)
                    {
                        m_wantedState = WantedState.FEED_PURPLE;
                        m_systemState = SystemState.SEARCHING_PURPLE;
                    }
                    else
                    {
                        m_wantedState = WantedState.FEED_GREEN;
                        m_systemState = SystemState.SEARCHING_GREEN;
                    }
                }
                else
                {
                    m_wantedState = WantedState.STAND_BY;
                }
                break;

            default:
                Dashboard.Telemetry_with_Text("Feeder", "can't run state machine with an unknown wanted state");
                break;
        }

        switch (m_systemState)
        {
            case IDLE:
                break;

            case SEARCHING_NEW_BALL:
                for (BallSlotSubsystem slot : m_feederSlots)
                {
                    if (slot.IsThereANewBall())
                    {
                        m_wantedState = WantedState.STAND_BY;
                        m_systemState = SystemState.IDLE;
                        break;
                    }
                }
                break;

            case SEARCHING_GREEN:
                for (int i = 0; i <=3; i++)
                {
                    if (m_feederSlots[i].getSlotState() == BallSlotSubsystem.SlotState.GREEN)
                    {
                        m_feedingSlotID = i;
                        m_systemState = SystemState.FEEDING_GREEN;
                    }
                }
                break;

            case SEARCHING_PURPLE:
                for (int i = 0; i <=3; i++)
                {
                    if (m_feederSlots[i].getSlotState() == BallSlotSubsystem.SlotState.PURPLE)
                    {
                        m_feedingSlotID = i;
                        m_systemState = SystemState.FEEDING_PURPLE;
                    }
                }
                break;

            case FEEDING_PURPLE:
            case FEEDING_GREEN:
                if (m_feederSlots[m_feedingSlotID].GetServoPos() !=  SlotServoFeedingPos)
                {
                    m_feederSlots[m_feedingSlotID].FeedBall();
                }
                else
                {
                    m_feederSlots[m_feedingSlotID].ReturnServoToHome();
                    m_wantedState = WantedState.STAND_BY;
                    m_systemState = SystemState.IDLE;
                    m_feedingSlotID = -1;
                }
                break;

            default:
                Dashboard.Telemetry_with_Text("Feeder", "can't run state machine with an unknown system state");
                break;
        }
    }

    /// Use this method to manually set the state of one slot (chosen in parameters, 0 for front, 1 for middle, 2 for back)
    public void SetSlotState(int slotID, BallSlotSubsystem.SlotState slotState)
    {
        if (slotID >= 0 && slotID <= 2)
        {
            m_feederSlots[slotID].SetSlotState(slotState);
        }
        else
        {
            Dashboard.Telemetry_with_Text("Feeder","SetSlotState used with an unknown ID");
        }
    }
}
