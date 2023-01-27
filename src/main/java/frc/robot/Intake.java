package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Intake {
    private double intakeSpeed = 0.25;
    private double outtakeSpeed = -0.15;
    private CANSparkMax leftIntake;
    private CANSparkMax rightIntake;
    public Intake(int leftIntakeid, int rightIntakeid) {
        leftIntake = new CANSparkMax(leftIntakeid, MotorType.kBrushless);
        rightIntake = new CANSparkMax(rightIntakeid, MotorType.kBrushless);
        rightIntake.setInverted(true);
    }
    public void intakeObject() {
        leftIntake.set(intakeSpeed);
        rightIntake.set(intakeSpeed);
    }
    public void outtakeObject() {
        leftIntake.set(outtakeSpeed);
        rightIntake.set(outtakeSpeed);
    }
    public void stopMotors() {
        leftIntake.set(0);
        rightIntake.set(0);
    }
}
