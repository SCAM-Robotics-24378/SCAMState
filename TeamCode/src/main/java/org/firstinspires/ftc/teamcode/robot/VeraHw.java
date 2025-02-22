package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap.DeviceMapping;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystem.SubsystemBase2;
import org.firstinspires.ftc.teamcode.subsystem.util.DenoiseUtility;

import java.util.List;

public class VeraHw extends SubsystemBase2 {

    private final List<LynxModule> allHubs;
    private final DeviceMapping<VoltageSensor> voltageSensor;
    private double loopTime = 0;
    private double loopTimeDenoise = 0;
    private final ElapsedTime voltageTimer = new ElapsedTime();
    private final DenoiseUtility denoiseFps = new DenoiseUtility(15);
    private final Telemetry telemetry;

    @lombok.Getter   // This alleviates the need for a getter method.
    private double voltage = 0;

    public VeraHw(List<LynxModule> allHubs,
                  DeviceMapping<VoltageSensor> voltageSensor, Telemetry telemetry) {
        this.allHubs = allHubs;
        this.voltageSensor = voltageSensor;

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        voltage = voltageSensor.iterator().next().getVoltage();
        voltageTimer.reset();
        this.telemetry = telemetry;
    }

    public void read() {
        if (voltageTimer.seconds() > 5) {
            voltageTimer.reset();
            voltage = voltageSensor.iterator().next().getVoltage();
        }
    }

    public void loop() {
        double loop = System.nanoTime();
        loopTimeDenoise = denoiseFps.filter(1e9 / (loop - loopTime));
        telemetry.addData("fps", df3.format(loopTimeDenoise));
        loopTime = loop;
    }

    public void write() {
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
    }

    public String getLoopTimeStr() {
        return df3.format(loopTimeDenoise);
    }
}
