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
    private boolean flywheelState;
    private enum ArtifactSystemStates {
        INTAKE,
        OUTTAKE,
        HUMAN_INTAKE,
        FLUSH,
        IDLE
    }
    private double leftPower;
    private double rightPower;
    private double intakePower;
    private double indexPower;
    private double flwPower;

    

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
        /*
        INPUT MAP:
        RT = Ground Intake
        RB = Human Intake
        LT = Outtake
        LB = Flush
        X = FLywheel Foreward
        B = Flywheel Stop
         */
        ArtifactSystemStates artifactSystemState = ArtifactSystemStates.IDLE;

        if (gamepad.isDown(GamepadKeys.Button.RIGHT_BUMPER)) {
            artifactSystemState = ArtifactSystemStates.HUMAN_INTAKE;
        } else if (gamepad.isDown(GamepadKeys.Button.LEFT_BUMPER)) {
            artifactSystemState = ArtifactSystemStates.FLUSH;
        } else if (gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.1) {
            artifactSystemState = ArtifactSystemStates.INTAKE;
        } else if (gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.1) {
            artifactSystemState = ArtifactSystemStates.OUTTAKE;
        }
        if (gamepad.wasJustPressed(GamepadKeys.Button.X)) {
            flywheelState = true;
        } else if (gamepad.wasJustPressed(GamepadKeys.Button.B)) {
            flywheelState = false;
        }
        switch(artifactSystemState) {
            case IDLE:
                intakePower = 0;
                indexPower = 0;
                leftPower = 0;
                rightPower = 0;
                flwPower = 0;
                break;
            case INTAKE:
                intakePower = 1;
                indexPower = 1;
                leftPower = 1;
                rightPower = -1;
                break;
            case HUMAN_INTAKE:
                leftPower = 1;
                rightPower = -1;
                flwPower = -0.5;
                indexPower = 0;
                intakePower = 0;
                break;
            case OUTTAKE:
                intakePower = 1;
                indexPower = 1;
                leftPower = -0.5;
                rightPower = 0.5;
                break;
            case FLUSH:
                leftPower = 1;
                rightPower = -1;
                intakePower = -1;
                indexPower = -1;
                break;
        }
        if (flywheelState) {
            flwPower = 1;
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
