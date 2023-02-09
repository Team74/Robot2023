// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.sql.DriverAction;

import javax.lang.model.util.ElementScanner14;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  AHRS gyro = new AHRS();
  SwerveModule[] swerveModules = {
    new SwerveModule(3, 4, 3, 71.1), //Front Left
    new SwerveModule(1, 2, 0, 30.6), //Front Right
    new SwerveModule(5, 6, 2, 335.8), //Back Left
    new SwerveModule(7, 8, 1, 184.7)  //Back Right
  };

  XboxController driveController = new XboxController(0);
  private Intake intake = new Intake(17,14);
  private DriveBase driveBase = new DriveBase(swerveModules, gyro);

  double speedMode;

  @Override
  public void robotInit() {
    driveBase.resetGyro();
    double xVelocity;
    double yVelocity;
    double rotationVelocity;
  }

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    speedMode = 2;
  }

  @Override
  public void teleopPeriodic() {
    double xVelocity = driveController.getLeftX();
    double yVelocity = -driveController.getLeftY();
    double rotationVelocity = driveController.getRightX();

    if(xVelocity < 0.05 && xVelocity > -0.05){
      xVelocity = 0;
    }
    if(yVelocity < 0.05 && yVelocity > -0.05){
      yVelocity = 0;
    }
    if(rotationVelocity < 0.05 && rotationVelocity > -0.05){
      rotationVelocity = 0;
    }

    if(driveController.getRightBumperPressed() & speedMode < 3){
      speedMode = speedMode + 1;
    } 
    if(driveController.getLeftBumperPressed() & speedMode > 1){
      speedMode = speedMode -                             1;
    }

    if(speedMode == 1){
      xVelocity = xVelocity * 1;
      yVelocity = yVelocity * 1;
      rotationVelocity = rotationVelocity * 100;
    } else if(speedMode == 2){
      xVelocity = xVelocity * 2;
      yVelocity = yVelocity * 2;
      rotationVelocity = rotationVelocity * 150;
    } else if(speedMode == 3){
      xVelocity = xVelocity * 3;
      yVelocity = yVelocity * 3;
      rotationVelocity = rotationVelocity * 200;
    } 

    if(driveController.getYButtonPressed()){
      driveBase.resetGyro();
    }

    driveBase.moveRobotFieldOriented(xVelocity, yVelocity, rotationVelocity);  //Meters per second & degrees per second

    SmartDashboard.putNumber("X Velocity", xVelocity);
    SmartDashboard.putNumber("Y Velocity", yVelocity);
    SmartDashboard.putNumber("R Velocity", rotationVelocity);


    intakeControl(driveController);

    driveBase.printSwerveSpeeds();
    driveBase.updatePIDLoops();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}

  public void intakeControl(XboxController controller) {
    boolean rightBumper = controller.getRightBumper();
    boolean leftBumper = controller.getLeftBumper();

    if(rightBumper) {
      intake.intakeObject();
    }else if(leftBumper){
      intake.outtakeObject();
    }else{
      intake.stopMotors();
    }
  }
}
