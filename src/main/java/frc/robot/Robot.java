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
  SwerveModule testSwerveModule = new SwerveModule(14, 16, 0, 268.0);
  XboxController driveController = new XboxController(0);
  private Intake intake = new Intake(1,2);

  @Override
  public void robotInit() {}

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    double driveControllerRightX = driveController.getRightX();
    double driveControllerRightY = driveController.getRightY();

    if(driveControllerRightX < 0.05 && driveControllerRightX > -0.05){
      driveControllerRightX = 0;
    }
    if(driveControllerRightY < 0.05 && driveControllerRightY > -0.05){
      driveControllerRightY = 0;
    }
    
    double targetAngle = Math.atan2(driveControllerRightY,driveControllerRightX);
    targetAngle = targetAngle*180/Math.PI;

    if(driveControllerRightY < 0){
      targetAngle = targetAngle + 90 + 360;
    }else if(driveControllerRightY == 0 && driveControllerRightX == 0){
      targetAngle = 0.0;
    }else{
      targetAngle = targetAngle + 90;
    }

    targetAngle = targetAngle % 360;
    
    testSwerveModule.goToAngle(targetAngle);
    testSwerveModule.setSwerveSpin(Math.sqrt(Math.pow(driveControllerRightX, 2) + Math.pow(driveControllerRightY, 2)));

    SmartDashboard.putNumber("Swerve Drive Speed", testSwerveModule.getDriveSpeed());
    SmartDashboard.putNumber("Swerve Angle", testSwerveModule.getSwerveAngle());
    SmartDashboard.putNumber("Target Angle", targetAngle);
    
    intakeControl(driveController);
  }
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
}
