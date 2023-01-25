package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule {
    
    CANSparkMax turnMotor;
    CANSparkMax driveMotor;
    DutyCycleEncoder swerveEncoder;
    double encoderOffset;

    PIDController anglePID;

    SwerveModule(int turnMotorID, int driveMotorID, int turnEncoderPort, double turnEncoderOffset){
        turnMotor = new CANSparkMax(turnMotorID, MotorType.kBrushless);
        driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        swerveEncoder = new DutyCycleEncoder(turnEncoderPort);
        encoderOffset = turnEncoderOffset;

        anglePID = new PIDController(1.0, 0.0, 0.0);

        anglePID.enableContinuousInput(0.0, 360.0);
        anglePID.setTolerance(5,10);
        anglePID.reset();

    }

    public double getSwerveAngle(){
        double angle = swerveEncoder.get();
        angle = angle * 360;
        angle = angle - encoderOffset;
        angle = angle % 360;
        SmartDashboard.putNumber("Current Angle", angle);
        return angle;
    }

    public void goToAngle(double desiredAngle){

        SmartDashboard.putNumber("Desired Angle", desiredAngle);
        double rotationMotorSpeed = anglePID.calculate(getSwerveAngle(), desiredAngle);
        
        rotationMotorSpeed = MathUtil.clamp(rotationMotorSpeed, -0.5, 0.5);

        if(anglePID.atSetpoint()){
            rotationMotorSpeed = 0.0;
        }

        turnMotor.set(rotationMotorSpeed);
        SmartDashboard.putNumber("Swerve Rotation Speed", rotationMotorSpeed);

    }

    public void setSwerveSpin(double swervePower){
        driveMotor.set(swervePower);
    }
}
