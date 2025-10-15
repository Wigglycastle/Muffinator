package org.firstinspires.ftc.teamcode.Main.Subsystems;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LightingSystem {
    private final RevBlinkinLedDriver ledDriver;
    public LightingSystem(HardwareMap hardwareMap) {
        ledDriver = hardwareMap.get(RevBlinkinLedDriver.class, "led");
    }

    public void PreGameLights() {
        ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_RAINBOW_PALETTE);
    }

    public void AutoLights() {
        ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_FOREST_PALETTE);
    }

    public void MidGameLights() {
        ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.SINELON_RAINBOW_PALETTE);
    }

}
