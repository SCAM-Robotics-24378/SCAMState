package org.firstinspires.ftc.teamcode.robot;

import android.os.Environment;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.subsystem.Drivetrain;

import lombok.Getter;


public class Vera extends Robot {

    @Getter

    public static Telemetry telemetry;
    public static boolean isAutonomous = false;
    public GamepadEx gamepadEx1;
    public GamepadEx gamepadEx2;
    private final VeraHw veraHw;
    public Drivetrain drivetrain;
    HardwareMap hardwareMap;
    String extStoragePath = Environment.getExternalStorageDirectory().getAbsolutePath();
    String allianceFilePath = String.format("%s/FIRST/data/alliance.txt", extStoragePath);


    // This constructor is intended for Autonomous Opmodes
    public Vera(Telemetry telemetry,
                HardwareMap hardwareMap,
                boolean useDashTelemetry) {
        Vera.telemetry = (useDashTelemetry ?
                new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()) :
                telemetry);
        this.hardwareMap = hardwareMap;

        veraHw = new VeraHw(
                hardwareMap.getAll(LynxModule.class),
                hardwareMap.voltageSensor,
                Vera.telemetry);
        isAutonomous = true;
        initAuto();
    }

    // This constructor is intended for TeleOp Opmodes
    public Vera(Telemetry telemetry,
                HardwareMap hardwareMap,
                boolean useDashTelemetry,
                GamepadEx gamepadEx1,
                GamepadEx gamepadEx2) {
        Vera.telemetry = (useDashTelemetry ?
                new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()) :
                telemetry);
        this.hardwareMap = hardwareMap;

        veraHw = new VeraHw(
                hardwareMap.getAll(LynxModule.class),
                hardwareMap.voltageSensor,
                Vera.telemetry);
        this.gamepadEx1 = gamepadEx1;
        this.gamepadEx2 = gamepadEx2;
        isAutonomous = false;
        Vera.telemetry = (useDashTelemetry ?
                new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()) :
                telemetry);
        initTele();
    }

    public void reset() {
        super.reset();
    }

    public void initTele() {
        drivetrain = new Drivetrain(
                new Motor(hardwareMap, "leftFront"),
                new Motor(hardwareMap, "leftRear"),
                new Motor(hardwareMap, "rightRear"),
                new Motor(hardwareMap, "rightFront"),
                hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint"),
                telemetry,
                gamepadEx1
        );
    }

    public void initAuto() {
    }

    public void read() {
        veraHw.read();
    }

    public void loop() {
        veraHw.loop();
    }

    public void write() {
        veraHw.write();
        telemetry.update();
    }
}
