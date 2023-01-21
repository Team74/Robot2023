package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class SwerveModule {
    
    CANSparkMax turnMotor;
    CANSparkMax driveMotor;
    DutyCycleEncoder swerveEncoder;
    double encoderOffset;

    SwerveModule(int turnMotorID, int driveMotorID, int turnEncoderPort, double turnEncoderOffset){
        turnMotor = new CANSparkMax(turnMotorID, MotorType.kBrushless);
        driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        swerveEncoder = new DutyCycleEncoder(turnEncoderPort);
        encoderOffset = turnEncoderOffset;
    }

    public double getSwerveAngle(){
        double angle = swerveEncoder.get();
        angle = angle * 360;
        angle = angle - encoderOffset;
        angle = angle % 360;
        return angle;
    }

    public void setSwerveSpin(double swervePower){
        driveMotor.set(swervePower);
    }
}
