package org.firstinspires.ftc.teamcode.Main.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

public class ArtifactSystem {
    private final Motor FlywheelMotor;
    private final Motor IntakeMotor;
    private final Motor IndexerMotor;
    private final ServoEx LeftIndex;
    private final ServoEx RightIndex;

    

    public ArtifactSystem(HardwareMap hardwareMap) {
        // Initialize Motors
        FlywheelMotor = new Motor(hardwareMap, "flywheelMotor", Motor.GoBILDA.RPM_312);
        IntakeMotor = new Motor(hardwareMap, "intakeMotor", Motor.GoBILDA.RPM_312);
        IndexerMotor = new Motor(hardwareMap, "indexerMotor", Motor.GoBILDA.RPM_312);

        //Initialize Servos
        LeftIndex = new ServoEx(hardwareMap, "leftIndexServo");
        RightIndex = new ServoEx(hardwareMap, "rightIndexServo");
    }

    public void ProcessInput(GamepadEx gamepad) {



    }
}
