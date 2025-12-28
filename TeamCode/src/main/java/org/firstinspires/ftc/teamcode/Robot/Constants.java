package org.firstinspires.ftc.teamcode.Robot;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Constants {
    public static double topShooterKP = 0.1, topShooterKI = 0.0, topShooterKD = 0.0;
    public static double bottomShooterKP = 0.1, bottomShooterKI = 0.0, bottomShooterKD = 0.0;
    public static double shooterVelocityTolerance = 1.0;

    public static double turresKP = 0.1, turresKI = 0.0, turresKD = 0.0;
    public static double turresOrientationTolerance = 0.0;

    public static double drivetrainForwardKP = 0.1, drivetrainForwardKI = 0.0, drivetrainForwardKD = 0.0;
    public static double drivetrainStrafeKP = 0.1, drivetrainStrafeKI = 0.0, drivetrainStrafeKD = 0.0;
    public static double drivetrainRotationKP = 0.1, drivetrainRotationKI = 0.0, drivetrainRotationKD = 0.0;
    public static double drivetrainForwardTolerance = 0.5, drivetrainStrafeTolerance = 0.5, drivetrainRotationTolerance = 0.5; //Length in cm and angle in degrees

    public static double drivetrainNominalVoltage = 11.0, intakeNominalVoltage = 11.0, turresNominalVoltage = 11.0, shooterNominalVoltage = 11.0;
}
