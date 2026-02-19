package org.firstinspires.ftc.teamcode.Main.Subsystems;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LightingSystem {
    private final RevBlinkinLedDriver ledDriver;
    public LightingSystem(HardwareMap hardwareMap) {
        ledDriver = hardwareMap.get(RevBlinkinLedDriver.class, "led");
    }

    public void SetLights(RevBlinkinLedDriver.BlinkinPattern pattern) {
        ledDriver.setPattern(pattern);
    }

}
