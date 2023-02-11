package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveBase {
    SwerveModule[] swerveModules;
    AHRS gyro;
    SwerveDriveKinematics driveKinematics;
    ChassisSpeeds robotSpeed;
    SwerveModuleState[] moduleStates;
    double forwardOffset = 0.38;
    double sideOffset = 0.21;

    DriveBase(SwerveModule[] swerveModules, AHRS gyro){
        this.gyro = gyro;
        this.swerveModules = swerveModules;
        driveKinematics = new SwerveDriveKinematics(
            new Translation2d(forwardOffset, sideOffset),    //Front Left
            new Translation2d(forwardOffset, -sideOffset),    //Front Right
            new Translation2d(-forwardOffset, sideOffset),    //Back Left
            new Translation2d(-forwardOffset, -sideOffset)     //Back Right
        );
    }

    public double getGyroAngle(){
        double angle = gyro.getAngle();
        angle = angle % 360;
        angle = angle + 360;
        angle = angle % 360;
        SmartDashboard.putNumber("Robot Angle", angle);
        return angle;
    }

    public double getRollAngle(){
        double angle = gyro.getRoll();
        SmartDashboard.putNumber("Robot Pitch", angle);
        return angle;
    }

    public void resetGyro(){
        gyro.reset();
    }

    public void printSwerveAngles(){
        for(int i = 1; i <= 4; i++){
            SmartDashboard.putNumber("Swerve Angle " + i, swerveModules[i - 1].getSwerveAngle());
        }
    }

    public void printSwerveSpeeds(){
        for(int i = 1; i <= 4; i++){
            SmartDashboard.putNumber("Swerve Speed " + i, swerveModules[i - 1].getDriveSpeed());
        }
    }

    public void updatePIDLoops(){
        for(int i = 0; i < 4; i++){
            swerveModules[i].updateDrivePID();
        }
    }

    public void moveRobotFieldOriented(double xVelocity, double yVelocity, double rotationVelocity){
        rotationVelocity = rotationVelocity * Math.PI / 180.0;
        robotSpeed = ChassisSpeeds.fromFieldRelativeSpeeds(yVelocity, -xVelocity, -rotationVelocity, Rotation2d.fromDegrees(-getGyroAngle()));
        moduleStates = driveKinematics.toSwerveModuleStates(robotSpeed);
        for(int i = 0; i < 4; i++){
            SwerveModuleState optimizedModuleState = SwerveModuleState.optimize(moduleStates[i], Rotation2d.fromDegrees(swerveModules[i].getSwerveAngle()));
            swerveModules[i].goToAngle(optimizedModuleState.angle.getDegrees());
            swerveModules[i].setDriveSpeed(optimizedModuleState.speedMetersPerSecond);
        }
    }

    public void moveRobotRobotOriented(double xVelocity, double yVelocity, double rotationVelocity){
        rotationVelocity = rotationVelocity * Math.PI / 180.0;
        robotSpeed = new ChassisSpeeds(yVelocity, -xVelocity, -rotationVelocity);
        moduleStates = driveKinematics.toSwerveModuleStates(robotSpeed);
        for(int i = 0; i < 4; i++){
            SwerveModuleState optimizedModuleState = SwerveModuleState.optimize(moduleStates[i], Rotation2d.fromDegrees(swerveModules[i].getSwerveAngle()));
            swerveModules[i].goToAngle(optimizedModuleState.angle.getDegrees());
            swerveModules[i].setDriveSpeed(optimizedModuleState.speedMetersPerSecond);
        }
    }

    public void autoBalance(){
        double velocity = 1;

        if(getGyroAngle() > 180){
            if(getRollAngle() < -5){
                velocity = -velocity;
            }else if(getRollAngle() <= 5){
                velocity = 0;
            }
        }else{
            if(getRollAngle() > 5){
                velocity = -velocity;
            }else if(getRollAngle() >= -5){
                velocity = 0;
            }
        }

        moveRobotFieldOriented(0.0, velocity, 0.0);
    }
}
