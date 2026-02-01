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
    private final CRServoEx IntakeServo;
    private boolean flywheelState;
    public enum ArtifactSystemStates {
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
    private double intakeServoPower;
    public ArtifactSystemStates currentState;
    public double flywheelRPM;

    

    public ArtifactSystem(HardwareMap hardwareMap) {
        // Initialize Motors
        FlywheelMotor = new Motor(hardwareMap, "flywheelMotor", Motor.GoBILDA.BARE);
        IntakeMotor = new Motor(hardwareMap, "intakeMotor", Motor.GoBILDA.RPM_312);
        IndexerMotor = new Motor(hardwareMap, "indexerMotor", Motor.GoBILDA.RPM_312);

        //Initialize Servos
        LeftIndex = new CRServoEx(hardwareMap, "leftIndexerServo");
        RightIndex = new CRServoEx(hardwareMap, "rightIndexerServo");
        IntakeServo = new CRServoEx(hardwareMap,"IntakeServo");
        FlywheelMotor.setRunMode(Motor.RunMode.VelocityControl);

        currentState = ArtifactSystemStates.IDLE;
        flywheelState = false;
    }

    public void setState(ArtifactSystemStates state) {
        currentState = state;
    }

    public void setFlywheel(boolean enabled) {
        flywheelState = enabled;
    }

    public void Update() {
        flywheelRPM = FlywheelMotor.getCorrectedVelocity();
        /*
        INPUT MAP:
        RT = Ground Intake
        RB = Human Intake
        LT = Outtake
        LB = Flush
        X = Flywheel Forward
        B = Flywheel Stop
         */
        switch(currentState) {
            case IDLE:
                intakePower = 0;
                indexPower = 0;
                leftPower = 0;
                rightPower = 0;
                flwPower = 0;
                intakeServoPower = 0;
                break;
            case INTAKE:
                intakePower = 1;
                indexPower = .8;
                leftPower = 1;
                rightPower = -1;
                intakeServoPower = 1;
                break;
            case HUMAN_INTAKE:
                leftPower = 1;
                rightPower = -1;
                flwPower = -0.5;
                indexPower = 0;
                intakePower = 0;
                intakeServoPower = 0;
                break;
            case OUTTAKE:
                intakePower = 1;
                indexPower = .8;
                leftPower = -0.5;
                rightPower = 0.5;
                intakeServoPower = 1;
                break;
            case FLUSH:
                leftPower = 1;
                rightPower = -1;
                intakePower = -1;
                indexPower = -1;
                intakeServoPower = -1;
                break;
        }
        if (flywheelState) {
            flwPower = .9;
        }
        FlywheelMotor.set(flwPower);
        LeftIndex.set(leftPower);
        RightIndex.set(rightPower);
        IntakeMotor.set(intakePower);
        IndexerMotor.set(indexPower);
        IntakeServo.set(intakeServoPower);
    }
    public void FlywheelPowerTo(double Power) {
        FlywheelMotor.set(Power);
    }
    public void StaggeredFeed() {
        IntakeMotor.set(1);
        IndexerMotor.set(1);
        LeftIndex.set(-0.25);
        RightIndex.set(0.25);
        IntakeServo.set(1);
    }
    public void StopFeed() {
        IntakeMotor.set(0);
        IndexerMotor.set(0);
        LeftIndex.set(-0);
        RightIndex.set(0);
        IntakeServo.set(0);
    }
}
