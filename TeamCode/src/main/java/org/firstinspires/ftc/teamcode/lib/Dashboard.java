package org.firstinspires.ftc.teamcode.lib;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

@Config
public class Dashboard {
    static FtcDashboard m_dashboard;

    public static void InitDashboard() {
        m_dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    public static void Telemetry_with_Text(String key, String text) {
        telemetry.addData(key, text);
    }

    public static void Telemetry_with_number(String key, double number) {
        telemetry.addData(key, number);
    }

    public static void TelemetryUpdate() {
        telemetry.update();
    }
}
