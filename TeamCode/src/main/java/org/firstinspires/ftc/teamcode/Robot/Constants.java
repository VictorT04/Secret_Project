package org.firstinspires.ftc.teamcode.Robot;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Constants {
    public static double topShooterKP = 0.1, topShooterKI = 0.0, topShooterKD = 0.0; //TUNEME
    public static double bottomShooterKP = 0.1, bottomShooterKI = 0.0, bottomShooterKD = 0.0; //TUNEME
    public static double shooterVelocityTolerance = 1.0; //TUNEME

    public static double turresKP = 0.1, turresKI = 0.0, turresKD = 0.0; //TUNEME
    public static double turresOrientationTolerance = 0.0; //TUNEME

    public static double drivetrainForwardKP = 0.1, drivetrainForwardKI = 0.0, drivetrainForwardKD = 0.0; //TUNEME
    public static double drivetrainStrafeKP = 0.1, drivetrainStrafeKI = 0.0, drivetrainStrafeKD = 0.0; //TUNEME
    public static double drivetrainRotationKP = 0.1, drivetrainRotationKI = 0.0, drivetrainRotationKD = 0.0; //TUNEME
    public static double drivetrainForwardTolerance = 0.5, drivetrainStrafeTolerance = 0.5, drivetrainRotationTolerance = 0.5; //Length in cm and angle in degrees //TUNEME

    public static double drivetrainNominalVoltage = 11.0, intakeNominalVoltage = 11.0, turresNominalVoltage = 11.0, shooterNominalVoltage = 11.0; //TUNEME

    public static double GreenBallRedValue = 30.0, GreenBallBlueValue = 10.0, GreenBallGreenValue = 170.0; //TUNEME
    public static double PurpleBallRedValue = 150.0, PurpleBallBlueValue = 150.0, PurpleBallGreenValue = 80.0; //TUNEME
    public static double ColorSensorTolerance = 5.0; //TUNEME
    public static double SlotServoFeedingPos = 0.7, SlotServoHomePos = 0.2; //TUNEME
}
