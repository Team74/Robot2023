package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
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

    RelativeEncoder driveEncoder;
    SparkMaxPIDController drivePID;

    PIDController anglePID;

    double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

    SwerveModule(int turnMotorID, int driveMotorID, int turnEncoderPort, double turnEncoderOffset){
        turnMotor = new CANSparkMax(turnMotorID, MotorType.kBrushless);
        driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        swerveEncoder = new DutyCycleEncoder(turnEncoderPort);
        encoderOffset = turnEncoderOffset;

        driveEncoder = driveMotor.getEncoder();
        drivePID = driveMotor.getPIDController();

        // PID coefficients
        kP = 0.000000; 
        kI = 0;
        kD = 0.000000; 
        kIz = 0; 
        kFF = 0.001050; 
        kMaxOutput = 0.95; 
        kMinOutput = -0.95;

        drivePID.setP(kP);
        drivePID.setI(kI);
        drivePID.setD(kD);
        drivePID.setIZone(kIz);
        drivePID.setFF(kFF);
        drivePID.setOutputRange(kMinOutput, kMaxOutput);

        SmartDashboard.putNumber("P Gain", kP);
        SmartDashboard.putNumber("I Gain", kI);
        SmartDashboard.putNumber("D Gain", kD);
        SmartDashboard.putNumber("I Zone", kIz);
        SmartDashboard.putNumber("Feed Forward", kFF);
        SmartDashboard.putNumber("Max Output", kMaxOutput);
        SmartDashboard.putNumber("Min Output", kMinOutput);

        driveMotor.burnFlash();

        anglePID = new PIDController(0.005, 0.0, 0.0001);

        anglePID.enableContinuousInput(0.0, 360.0);
        anglePID.setTolerance(4,2);
        anglePID.reset();

    }

    public double getDriveSpeed(){
        double speed = driveEncoder.getVelocity();
        speed = speed * 13 * 29 / 108.0 / 60 / 100;
        return speed;
    }

    public double getSwerveAngle(){
        double angle = swerveEncoder.get();
        angle = angle * 360;
        angle = angle - encoderOffset;
        angle = angle % 360;
        angle = (angle + 360) % 360;
        return angle;
    }

    public void setRotationPower(double power){
        turnMotor.set(power);
    }

    public void goToAngle(double desiredAngle){
        double rotationMotorSpeed = anglePID.calculate(getSwerveAngle(), desiredAngle);
        
        rotationMotorSpeed = MathUtil.clamp(rotationMotorSpeed, -0.75, 0.75);

        if(anglePID.atSetpoint()){
            rotationMotorSpeed = 0.0;
        }

        turnMotor.set(rotationMotorSpeed);
    }

    public void setDrivePower(double swervePower){
        driveMotor.set(swervePower);
    }

    public void updateDrivePID(){
        // read PID coefficients from SmartDashboard
        double p = SmartDashboard.getNumber("P Gain", 0);
        double i = SmartDashboard.getNumber("I Gain", 0);
        double d = SmartDashboard.getNumber("D Gain", 0);
        double iz = SmartDashboard.getNumber("I Zone", 0);
        double ff = SmartDashboard.getNumber("Feed Forward", 0);
        double max = SmartDashboard.getNumber("Max Output", 0);
        double min = SmartDashboard.getNumber("Min Output", 0);

        // if PID coefficients on SmartDashboard have changed, write new values to controller
        if((p != kP)) { drivePID.setP(p); kP = p; }
        if((i != kI)) { drivePID.setI(i); kI = i; }
        if((d != kD)) { drivePID.setD(d); kD = d; }
        if((iz != kIz)) { drivePID.setIZone(iz); kIz = iz; }
        if((ff != kFF)) { drivePID.setFF(ff); kFF = ff; }
        if((max != kMaxOutput) || (min != kMinOutput)) { 
        drivePID.setOutputRange(min, max); 
        kMinOutput = min; kMaxOutput = max; 
        }
    }

    public void setDriveSpeed(double swerveSpeed){
        swerveSpeed = swerveSpeed * 108 * 60 * 100 / 13.0 / 29.0;

       drivePID.setReference(swerveSpeed, CANSparkMax.ControlType.kVelocity);
    }
    
}
