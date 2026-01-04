package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Robot.RobotContainer;

import java.util.List;

public class Camera {
    private Limelight3A m_camera;
    private LLResult m_cameraResults;

    private RobotContainer.RobotMode robotMode;

    public Camera(HardwareMap hmap, RobotContainer.RobotMode robotMode)
    {
        m_camera = hmap.get(Limelight3A.class,"Cam_ille");
        m_camera.pipelineSwitch(0); //Obelisk pipeline
        m_camera.setPollRateHz(100);
    }

    /// Try to identify the obelisk.
    /// If it doesn't succeed to identify it it returns 0
    /// If the obelisk is recognize it returns the ID of the obelisk Tag and change automatically to the correct shoot pipeline according to the robot mode
    public int IdentifyObelisk()
    {
        if (m_cameraResults != null)
        {
            if (robotMode == RobotContainer.RobotMode.AUTO_BLUE || robotMode == RobotContainer.RobotMode.TELEOP_BLUE)
            {
                m_camera.pipelineSwitch(1);
            }
            else
            {
                m_camera.pipelineSwitch(2);
            }
            return m_cameraResults.getFiducialResults().get(0).getFiducialId();
        }
        return 0;
    }

    public double GetTargetTx()
    {
        if (m_cameraResults != null)
        {
            return m_cameraResults.getTx();
        }
        return 90.0;
    }

    public double GetTargetTy()
    {
        if (m_cameraResults != null)
        {
            return m_cameraResults.getTy();
        }
        return 90.0;
    }

    public boolean IsSeeingGoal()
    {
        if (m_cameraResults != null)
        {
            return (m_cameraResults.getPipelineIndex() != 0);
        }
        return false;
    }

    public void UpdateCameraResults()
    {
        LLResult currentResult = m_camera.getLatestResult();
        if (currentResult.isValid())
        {
            m_cameraResults = currentResult;
        }
    }
}
