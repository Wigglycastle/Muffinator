package org.firstinspires.ftc.teamcode.Main.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

public class Climb {
    private final Motor ClimbMotor;

    public Climb(HardwareMap hardwareMap) {
        ClimbMotor = new Motor(hardwareMap, "climb", Motor.GoBILDA.RPM_84);
    }

    public void ProcessInput(GamepadEx gamepad) {

    }
}
