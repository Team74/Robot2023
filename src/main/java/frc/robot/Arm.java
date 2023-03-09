package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Arm {

    CANSparkMax shoulderMotor;
    PIDController shoulderPID;
    DutyCycleEncoder shoulderEncoder;

    double kP, kI, kD, kFF;

    double encoderOffset = 111.15;
    double lastSavedAngle;
    double lastTargetPosition;

    Arm(int shoulderMotorID, int shoulderEncoderPort){

        shoulderMotor = new CANSparkMax(shoulderMotorID, MotorType.kBrushless);
        shoulderEncoder = new DutyCycleEncoder(shoulderEncoderPort);

        lastSavedAngle = 0.0;
        lastTargetPosition = 0.0;

        // PID coefficients
        kP = 0.010000; 
        kI = 0.000100;
        kD = 0.000000; 
        kFF = 0.001500; 

        SmartDashboard.putNumber("P Gain", kP);
        SmartDashboard.putNumber("I Gain", kI);
        SmartDashboard.putNumber("D Gain", kD);
        SmartDashboard.putNumber("I Zone", 0.0);
        SmartDashboard.putNumber("Feed Forward", kFF);
        SmartDashboard.putNumber("Max Output", 0.0);
        SmartDashboard.putNumber("Min Output", 0.0);

        shoulderPID = new PIDController(kP, kI, kD);

        shoulderPID.setTolerance(0.5,5.0);
        shoulderPID.reset();
    }

    public void setStart(){
        lastSavedAngle = 0.0;
        lastTargetPosition = 0.0;
    }

    public void updateShoulderPID(){
        // read PID coefficients from SmartDashboard
        double p = SmartDashboard.getNumber("P Gain", 0);
        double i = SmartDashboard.getNumber("I Gain", 0);
        double d = SmartDashboard.getNumber("D Gain", 0);
        double ff = SmartDashboard.getNumber("Feed Forward", 0);

        // if PID coefficients on SmartDashboard have changed, write new values to controller
        if((p != kP)) { shoulderPID.setP(p); kP = p; }
        if((i != kI)) { shoulderPID.setI(i); kI = i; }
        if((d != kD)) { shoulderPID.setD(d); kD = d; }
        if((ff != kFF)) { kFF = ff;}
    }

    public double getShoulderPosition(){
        double angle = shoulderEncoder.get();

        angle = angle * 120;
        angle = angle + encoderOffset;
        angle = angle % 120;
        angle = (angle + 120) % 120;
        angle = (-1.0* angle) + 120;
        if(angle > 115){
            angle = angle - 120;
        }

        SmartDashboard.putNumber("Shoulder Position", angle);

        lastSavedAngle = angle;
        return angle;
    }

    public void moveShoulder(double rotationPower){
        shoulderMotor.set(rotationPower);
    }

    public void setShoulderPosition(double targetPosition){
        if(targetPosition < lastTargetPosition - 1){
            targetPosition = lastTargetPosition - 1;
        }else if(targetPosition > lastTargetPosition + 1){
            targetPosition = lastTargetPosition + 1;
        }

        SmartDashboard.putNumber("Target Shoulder", targetPosition);
        lastTargetPosition = targetPosition;

        double shoulderMotorPower = shoulderPID.calculate(getShoulderPosition(), targetPosition);
                
        if(shoulderPID.atSetpoint()){
            shoulderMotorPower = 0.0;
        }

        shoulderMotorPower = MathUtil.clamp(shoulderMotorPower, -0.4, 0.4);

        shoulderMotorPower += kFF * targetPosition;

        SmartDashboard.putNumber("Shoulder Power", shoulderMotorPower);

        shoulderMotor.set(shoulderMotorPower);
    }

    public void setShoulderPower(double power){
        shoulderMotor.set(power);
    }

}
