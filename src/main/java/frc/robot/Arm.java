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

    double encoderOffset = -1.72;
    double lastSavedAngle;
    double lastTargetPosition;
    double lastShoulderPosition;
    double armSpeed = 1;

    Arm(int shoulderMotorID, int shoulderEncoderPort){

        shoulderMotor = new CANSparkMax(shoulderMotorID, MotorType.kBrushless);
        shoulderEncoder = new DutyCycleEncoder(shoulderEncoderPort);

        lastSavedAngle = 0.0;
        lastTargetPosition = 0.0;
        lastShoulderPosition = 0.0;

        // PID coefficients
        kP = 0.010000; 
        kI = 0.000500;
        kD = 0.000000; 
        kFF = 0.000600; 

        /*SmartDashboard.putNumber("P Gain", kP);
        SmartDashboard.putNumber("I Gain", kI);
        SmartDashboard.putNumber("D Gain", kD);
        SmartDashboard.putNumber("I Zone", 0.0);
        SmartDashboard.putNumber("Feed Forward", kFF);
        SmartDashboard.putNumber("Max Output", 0.0);
        SmartDashboard.putNumber("Min Output", 0.0);*/

        shoulderPID = new PIDController(kP, kI, kD);

        shoulderPID.setTolerance(0.0,1.0);
        shoulderPID.reset();
    }

    public void setStart(){
        lastSavedAngle = getShoulderPosition();
        lastTargetPosition = getShoulderPosition();
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
        if(angle > 110){
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
        armSpeed = 4 * Math.abs(lastShoulderPosition-getShoulderPosition());

        armSpeed = MathUtil.clamp(armSpeed, 1.5, 5);
        
        if(targetPosition < lastTargetPosition - armSpeed){
            targetPosition = lastTargetPosition - armSpeed;
        }else if(targetPosition > lastTargetPosition + armSpeed){
            targetPosition = lastTargetPosition + armSpeed;
        }

        SmartDashboard.putNumber("Target Shoulder", targetPosition);
        lastTargetPosition = targetPosition;
        lastShoulderPosition = getShoulderPosition();

        double i;
        double p;

        if(Math.abs(getShoulderPosition() - targetPosition) < 2){
            i = 0.000500;
            p = 0.100000;
        }else if(Math.abs(getShoulderPosition() - targetPosition) < 10){
            i = 0.000500;
            p = 0.020000;
        }else{
            i = 0.000500;
            p = 0.010000;
        }

        if(i != kI){
            kI = i;
            shoulderPID.setI(i);
        }

        if(p != kP){
            kP = p;
            shoulderPID.setP(p);
        }

        double shoulderMotorPower = shoulderPID.calculate(getShoulderPosition(), targetPosition);
                
        if(shoulderPID.atSetpoint()){
            shoulderMotorPower = 0.0;
        }

        shoulderMotorPower = MathUtil.clamp(shoulderMotorPower, -0.4, 0.4);

        shoulderMotorPower += kFF * targetPosition;

        if(shoulderMotorPower < 0.05 && shoulderMotorPower > -0.05){
            shoulderMotorPower = 0.0;
        }

        SmartDashboard.putNumber("Shoulder Power", shoulderMotorPower);

        shoulderMotor.set(shoulderMotorPower);
    }

    public void setShoulderPower(double power){
        shoulderMotor.set(power);
    }

}
