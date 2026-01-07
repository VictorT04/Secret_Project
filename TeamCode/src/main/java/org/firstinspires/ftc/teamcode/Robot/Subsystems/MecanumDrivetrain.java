package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import static org.firstinspires.ftc.teamcode.Robot.Constants.drivetrainForwardKP;
import static org.firstinspires.ftc.teamcode.Robot.Constants.drivetrainForwardKI;
import static org.firstinspires.ftc.teamcode.Robot.Constants.drivetrainForwardKD;
import static org.firstinspires.ftc.teamcode.Robot.Constants.drivetrainForwardTolerance;

import static org.firstinspires.ftc.teamcode.Robot.Constants.drivetrainStrafeKP;
import static org.firstinspires.ftc.teamcode.Robot.Constants.drivetrainStrafeKI;
import static org.firstinspires.ftc.teamcode.Robot.Constants.drivetrainStrafeKD;
import static org.firstinspires.ftc.teamcode.Robot.Constants.drivetrainStrafeTolerance;

import static org.firstinspires.ftc.teamcode.Robot.Constants.drivetrainRotationKP;
import static org.firstinspires.ftc.teamcode.Robot.Constants.drivetrainRotationKI;
import static org.firstinspires.ftc.teamcode.Robot.Constants.drivetrainRotationKD;
import static org.firstinspires.ftc.teamcode.Robot.Constants.drivetrainRotationTolerance;

import static org.firstinspires.ftc.teamcode.Robot.Constants.drivetrainNominalVoltage;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.teamcode.Robot.RobotContainer;
import org.firstinspires.ftc.teamcode.lib.PidRBL;

public class MecanumDrivetrain {
      private final DcMotor m_bottomLeft, m_bottomRight, m_topLeft, m_topRight;
      private double m_bottomLeftPower, m_bottomRightPower, m_topLeftPower, m_topRightPower;

      private boolean m_fieldOrientedActivated;
      private final SparkFunOTOS m_otos;
      private SparkFunOTOS.Pose2D m_robotPos;
      private PidRBL m_forwardPIDController, m_strafePIDController, m_rotationPIDController;

      private RobotContainer robot;

      MecanumDrivetrain(HardwareMap hmap, RobotContainer robot)
      {
          m_bottomLeft = hmap.get(DcMotor.class, "bottomLeftMotor");
          m_bottomRight = hmap.get(DcMotor.class, "bottomRightMotor");
          m_topLeft = hmap.get(DcMotor.class, "topLeftMotor");
          m_topRight = hmap.get(DcMotor.class, "topRightMotor");

          m_bottomLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
          m_bottomLeft.setDirection(DcMotorSimple.Direction.FORWARD);
          m_bottomLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

          m_bottomRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
          m_bottomRight.setDirection(DcMotorSimple.Direction.FORWARD);
          m_bottomRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

          m_topLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
          m_topLeft.setDirection(DcMotorSimple.Direction.REVERSE);
          m_topLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

          m_topRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
          m_topRight.setDirection(DcMotorSimple.Direction.REVERSE);
          m_topRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

          m_otos = hmap.get(SparkFunOTOS.class, "SparkFunOTOS");

          m_otos.setLinearUnit(DistanceUnit.CM);
          m_otos.setAngularUnit(AngleUnit.RADIANS);
          m_otos.setOffset(new SparkFunOTOS.Pose2D(0,10,0)); //TUNEME
          m_otos.calibrateImu();

          m_forwardPIDController = new PidRBL (drivetrainForwardKP, drivetrainForwardKI, drivetrainForwardKD);
          m_forwardPIDController.SetTolerance(drivetrainForwardTolerance);

          m_strafePIDController = new PidRBL (drivetrainStrafeKP, drivetrainStrafeKI, drivetrainStrafeKD);
          m_strafePIDController.SetTolerance(drivetrainStrafeTolerance);

          m_rotationPIDController= new PidRBL (drivetrainRotationKP, drivetrainRotationKI, drivetrainRotationKD);
          m_rotationPIDController.SetTolerance(drivetrainRotationTolerance);
          m_rotationPIDController.SetInputLimits(true);
          m_rotationPIDController.SetInputLimits(0,Math.PI*2);
          m_rotationPIDController.SetContinuous(true);

          this.robot = robot;
      }

      public void ResetOtosTracking()
      {
          m_otos.resetTracking();
          ActualiseRobotPos();
      }

      public void ActualiseRobotPos()
      {
          m_robotPos = m_otos.getPosition();
      }

      public void ToggleFieldOriented()
      {
        m_fieldOrientedActivated = !m_fieldOrientedActivated;
      }

      public void TeleopDrivetrain(double forward, double strafe, double rotation)
      {
          if (m_fieldOrientedActivated)
          {
              double forwardCopy = forward;
              forward = forward * Math.cos(m_robotPos.h) + strafe * Math.sin(m_robotPos.h);
              strafe = -forwardCopy * Math.sin(m_robotPos.h) + strafe * Math.cos(m_robotPos.h);
          }

          m_bottomLeftPower = forward - strafe + rotation;
          m_bottomRightPower = forward + strafe - rotation;
          m_topLeftPower = forward + strafe + rotation;
          m_topRightPower = forward - strafe - rotation;

          SetMotorPower();
      }

      public boolean AutoLine(double targetX, double targetY, double targetHeading)
      {
          ActualiseRobotPos();
          double dX = targetX - m_robotPos.x;
          double dY = targetY - m_robotPos.y;

          double forwardPower = m_forwardPIDController.Calculate(Math.cos(m_robotPos.h)*dX + Math.sin(m_robotPos.h)*dY,0);
          double strafePower = m_strafePIDController.Calculate(Math.cos(m_robotPos.h)*dY - Math.sin(m_robotPos.h)*dX,0);
          double rotationPower = m_rotationPIDController.Calculate(targetHeading*Math.PI/180,m_robotPos.h);

          m_bottomLeftPower = forwardPower + strafePower - rotationPower;
          m_bottomRightPower = forwardPower - strafePower + rotationPower;
          m_topLeftPower = forwardPower - strafePower - rotationPower;
          m_topRightPower = forwardPower + strafePower + rotationPower;

          m_bottomLeftPower = GetVoltageCompensed(m_bottomLeftPower);
          m_bottomRightPower = GetVoltageCompensed(m_bottomRightPower);
          m_topLeftPower = GetVoltageCompensed(m_topLeftPower);
          m_topRightPower = GetVoltageCompensed(m_topRightPower);

          return SetMotorPower() < 0.05;
      }

    public boolean AutoXSpline(double targetX, double targetY, double targetHeading, double splineProgression)
    {
        ActualiseRobotPos();
        double dX = targetX - m_robotPos.x;
        double dY = targetY - m_robotPos.y;
        double Xcoef = 1.0-splineProgression, Ycoef = splineProgression;

        double forwardPower = m_forwardPIDController.Calculate(Math.cos(m_robotPos.h)*dX*Xcoef + Math.sin(m_robotPos.h)*dY*Ycoef,0);
        double strafePower = m_strafePIDController.Calculate(Math.cos(m_robotPos.h)*dY*Ycoef - Math.sin(m_robotPos.h)*dX*Xcoef,0);
        double rotationPower = m_rotationPIDController.Calculate(targetHeading*Math.PI/180,m_robotPos.h);

        m_bottomLeftPower = forwardPower + strafePower - rotationPower;
        m_bottomRightPower = forwardPower - strafePower + rotationPower;
        m_topLeftPower = forwardPower - strafePower - rotationPower;
        m_topRightPower = forwardPower + strafePower + rotationPower;

        m_bottomLeftPower = GetVoltageCompensed(m_bottomLeftPower);
        m_bottomRightPower = GetVoltageCompensed(m_bottomRightPower);
        m_topLeftPower = GetVoltageCompensed(m_topLeftPower);
        m_topRightPower = GetVoltageCompensed(m_topRightPower);

        return SetMotorPower() < 0.05;
    }

    public boolean AutoYSpline(double targetX, double targetY, double targetHeading, double splineProgression)
    {
        ActualiseRobotPos();
        double dX = targetX - m_robotPos.x;
        double dY = targetY - m_robotPos.y;
        double Ycoef = 1.0-splineProgression, Xcoef = splineProgression;

        double forwardPower = m_forwardPIDController.Calculate(Math.cos(m_robotPos.h)*dX*Xcoef + Math.sin(m_robotPos.h)*dY*Ycoef,0);
        double strafePower = m_strafePIDController.Calculate(Math.cos(m_robotPos.h)*dY*Ycoef - Math.sin(m_robotPos.h)*dX*Xcoef,0);
        double rotationPower = m_rotationPIDController.Calculate(targetHeading*Math.PI/180,m_robotPos.h);

        m_bottomLeftPower = forwardPower + strafePower - rotationPower;
        m_bottomRightPower = forwardPower - strafePower + rotationPower;
        m_topLeftPower = forwardPower - strafePower - rotationPower;
        m_topRightPower = forwardPower + strafePower + rotationPower;

        m_bottomLeftPower = GetVoltageCompensed(m_bottomLeftPower);
        m_bottomRightPower = GetVoltageCompensed(m_bottomRightPower);
        m_topLeftPower = GetVoltageCompensed(m_topLeftPower);
        m_topRightPower = GetVoltageCompensed(m_topRightPower);

        return SetMotorPower() < 0.05;
    }

      private double GetVoltageCompensed(double value)
      {
          return robot.GetVoltageSensorValue()*value/drivetrainNominalVoltage;
      }

      /// Check if any wanted power is greater than 1 or smaller than 1 and reduces it if needed keeping proportionality.
      /// Then apply the 4 variables for MotorPower to the correct motor
      /// Don't use voltage compensation
      ///
      /// @Return The value of the greatest power apply between all motors
      public double SetMotorPower()
      {
          double max = JavaUtil.maxOfList(JavaUtil.createListWith(Math.abs(m_bottomLeftPower),Math.abs(m_bottomRightPower), Math.abs(m_topLeftPower), Math.abs(m_topRightPower)));
          if (max > 1)
          {
              m_bottomLeftPower /= max;
              m_bottomRightPower /= max;
              m_topLeftPower /= max;
              m_topRightPower /= max;
          }

          m_bottomLeft.setPower(m_bottomLeftPower);
          m_bottomRight.setPower(m_bottomRightPower);
          m_topLeft.setPower(m_topLeftPower);
          m_topRight.setPower(m_topRightPower);

          return max;
      }
}
