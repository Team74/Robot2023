package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class Arm {

    CANSparkMax elbowMotor;
    PIDController elbowPID;
    DutyCycleEncoder elbowEncoder;

    
    Arm(int elbowMotorID, int elbowEncoderPort){

        elbowMotor = new CANSparkMax(elbowMotorID, MotorType.kBrushless);
        elbowEncoder = new DutyCycleEncoder(elbowEncoderPort);

        elbowPID = new PIDController(0.0, 0.0, 0.0);

        elbowPID.setTolerance(0,0);
        elbowPID.reset();
    }

    public double getElbowPosition(){
        double angle = elbowEncoder.get();
        return angle;
    }

    public void moveElbow(double rotationPower){
        elbowMotor.set(rotationPower);
    }

    public void setElbowPosition(double targetPosition, boolean noPower){
        double elbowMotorPower = elbowPID.calculate(getElbowPosition(), targetPosition);
        
        elbowMotorPower = MathUtil.clamp(elbowMotorPower, -0.75, 0.75);
        
        if(elbowPID.atSetpoint()){
            if(noPower){
                elbowMotorPower = 0.0;
            }else{
                elbowMotorPower = 0.05;
            }
        }

        elbowMotor.set(elbowMotorPower);
    }

    public void setElbowLevel(int elbowLevel){
        double targetPosition = 0.0;

        switch (elbowLevel) {
            case 1: targetPosition = 10.0;
                    break;

            case 2: targetPosition = 20.0;
                    break;

            case 3: targetPosition = 30.0;
                    break;

            case 4: targetPosition = 40.0;
                    break;

            case 5: targetPosition = 50.0;
                    break;

            case 6: targetPosition = 60.0;
                    break;

            case 7: targetPosition = 70.0;
                    break;

            case 8: targetPosition = 80.0;
                    break;
        }

        setElbowPosition(targetPosition, false);
    }

}
