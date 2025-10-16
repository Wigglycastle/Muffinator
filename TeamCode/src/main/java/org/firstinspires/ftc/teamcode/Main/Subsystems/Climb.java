package org.firstinspires.ftc.teamcode.Main.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

public class Climb {
    private final Motor ClimbMotor;
    private int targetPos;

    public Climb(HardwareMap hardwareMap) {
        // Setup and zero climb
        ClimbMotor = new Motor(hardwareMap, "climb", Motor.GoBILDA.RPM_84);
        ClimbMotor.setRunMode(Motor.RunMode.PositionControl);
        ClimbMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        ClimbMotor.resetEncoder();
    }

    public void EnableMotors() {
        ClimbMotor.set(1);
    }

    public void ProcessInput(GamepadEx gamepad) {
        if (gamepad.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
            targetPos = 10;
        }
        if (gamepad.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            targetPos = 0;
        }

        ClimbMotor.setTargetPosition(targetPos);

    }
}
