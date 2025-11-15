package org.firstinspires.ftc.teamcode.Main.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

public class ArtifactSystem {
    private final Motor FlywheelMotor;
    private final Motor IntakeMotor;
    private final Motor IndexerMotor;
    private final CRServoEx LeftIndex;
    private final CRServoEx RightIndex;
    private int flywheelState = 0;

    

    public ArtifactSystem(HardwareMap hardwareMap) {
        // Initialize Motors
        FlywheelMotor = new Motor(hardwareMap, "flywheelMotor", Motor.GoBILDA.RPM_312);
        IntakeMotor = new Motor(hardwareMap, "intakeMotor", Motor.GoBILDA.RPM_312);
        IndexerMotor = new Motor(hardwareMap, "indexerMotor", Motor.GoBILDA.RPM_312);

        //Initialize Servos
        LeftIndex = new CRServoEx(hardwareMap, "leftIndexerServo");
        RightIndex = new CRServoEx(hardwareMap, "rightIndexerServo");
    }

    public void ProcessInput(GamepadEx gamepad) {
        double leftPower = 0;
        double rightPower = 0;
        double intakePower = 0;
        double indexPower = 0;
        double flwPower = 0;
        if (gamepad.wasJustPressed(GamepadKeys.Button.X)) {
            flywheelState = 1;
        } else if (gamepad.wasJustPressed(GamepadKeys.Button.Y)) {
            flywheelState = 2;
        } else if (gamepad.wasJustPressed(GamepadKeys.Button.B)) {
            flywheelState = 0;
        }
        if (gamepad.isDown(GamepadKeys.Button.RIGHT_BUMPER)) {
            leftPower = 1;
            rightPower = -1;
            flwPower = -0.5;
        } else if (gamepad.isDown(GamepadKeys.Button.LEFT_BUMPER)) {
            leftPower = 1;
            rightPower = -1;
            intakePower = -1;
            indexPower = -1;
        }
        if (gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.1) {
            intakePower = 1;
            indexPower = 1;
            leftPower = 1;
            rightPower = -1;
        } else if (gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.1) {
            intakePower = 1;
            indexPower = 1;
            leftPower = -1;
            rightPower = 1;
        }
        if (flywheelState == 1) {
            flwPower = 1;
        } else if (flywheelState == 2) {
            flwPower = -1;
        }
        FlywheelMotor.set(flwPower);
        LeftIndex.set(leftPower);
        RightIndex.set(rightPower);
        IntakeMotor.set(intakePower);
        IndexerMotor.set(indexPower);
    }
    public void FlywheelPowerTo(double Power) {
        FlywheelMotor.set(Power);
    }
    public void StaggeredFeed() {
        IntakeMotor.set(1);
        IndexerMotor.set(1);
        LeftIndex.set(-0.25);
        RightIndex.set(0.25);
    }
    public void StopFeed() {
        IntakeMotor.set(0);
        IndexerMotor.set(0);
        LeftIndex.set(-0);
        RightIndex.set(0);
    }
}
