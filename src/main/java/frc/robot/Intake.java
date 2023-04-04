package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;

public class Intake {
    private double intakeSpeed = 0.5;
    private double outtakeSpeed = -0.5;
    private boolean limitOverride = false;
    private CANSparkMax leftIntake;
    private CANSparkMax rightIntake;
    private DigitalInput intakeLimit;

    public Intake(int leftIntakeid, int rightIntakeid, int limitPort) {
        leftIntake = new CANSparkMax(leftIntakeid, MotorType.kBrushed);
        rightIntake = new CANSparkMax(rightIntakeid, MotorType.kBrushed);
        rightIntake.setInverted(false);

        intakeLimit = new DigitalInput(limitPort);
        limitOverride = false;
    }

    public void intakeObject() {
        if(!intakeLimit.get() && !limitOverride){
            stopMotors();
        }else{
            leftIntake.set(intakeSpeed);
            rightIntake.set(intakeSpeed);
        }
    }

    public boolean intakeLimit(){
        return !intakeLimit.get();
    }

    public void setLimitOveride(boolean override){
        limitOverride = override;
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
