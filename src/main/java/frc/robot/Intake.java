package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;

public class Intake {
    private double intakeSpeed = 0.5;
    private double outtakeSpeed = -0.5;
    private CANSparkMax leftIntake;
    private CANSparkMax rightIntake;
    private DigitalInput intakeLimit;

    public Intake(int leftIntakeid, int rightIntakeid, int limitPort) {
        leftIntake = new CANSparkMax(leftIntakeid, MotorType.kBrushed);
        rightIntake = new CANSparkMax(rightIntakeid, MotorType.kBrushed);
        rightIntake.setInverted(false);

        intakeLimit = new DigitalInput(limitPort);
    }

    public void intakeObject() {
        if(intakeLimit.get()){
            stopMotors();
        }else{
            leftIntake.set(intakeSpeed);
            rightIntake.set(intakeSpeed);
        }
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
