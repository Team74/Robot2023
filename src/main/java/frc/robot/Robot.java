// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.sql.DriverAction;

import javax.lang.model.util.ElementScanner14;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.AxisCamera;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
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
    new SwerveModule(14, 16, 0, 67.7), //Front Left
    new SwerveModule(19, 18, 1, 101.2), //Front Right
    new SwerveModule(17 , 12, 2, 44.8), //Back Left
    new SwerveModule(10, 11, 3, 354.4)  //Back Right
  };

  XboxController driveController = new XboxController(0);
  XboxController opController = new XboxController(1);

  private Intake intake = new Intake(5, 6, 9);
  private Arm arm = new Arm(4, 4);
  private DriveBase driveBase = new DriveBase(swerveModules, gyro);

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

  int speedMode;

  double elbowPosition = 0;

  boolean opPOVPressed = false;

  boolean armOveride = false;

  @Override
  public void robotInit() {
    UsbCamera camera = CameraServer.startAutomaticCapture(0);

    driveBase.resetGyro();
    double xVelocity;
    double yVelocity;
    double rotationVelocity;

    elbowPosition = 0;
    arm.setStart();
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
    elbowPosition = 0;
    arm.setStart();
    armOveride = false;
  }

  @Override
  public void teleopPeriodic() {

    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");

    //NetworkTableEntry ledMode = table.getEntry("ledMode");
    //ledMode.setNumber(0);      // 1 = force off,   0 = follow pipeline

    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);

    double xVelocity = driveController.getLeftX();
    double yVelocity = -driveController.getLeftY();
    double rotationVelocity = driveController.getRightX();

    double shoulderPower = opController.getRightY();

    if(xVelocity < 0.05 && xVelocity > -0.05){
      xVelocity = 0;
    }
    if(yVelocity < 0.05 && yVelocity > -0.05){
      yVelocity = 0;
    }
    if(rotationVelocity < 0.05 && rotationVelocity > -0.05){
      rotationVelocity = 0;
    }

    if(shoulderPower < 0.05 && shoulderPower > -0.05){
      shoulderPower = 0;
    }

    if(driveController.getRightBumperPressed() & speedMode < 3){
      speedMode = speedMode + 1;
    } 
    if(driveController.getLeftBumperPressed() & speedMode > 0){
      speedMode = speedMode - 1;
    }

    if(speedMode == 0){
      xVelocity = xVelocity * 0.5;
      yVelocity = yVelocity * 0.5;
      rotationVelocity = rotationVelocity * 75;
    }else if(speedMode == 1){
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

    shoulderPower = shoulderPower * 0.35;

    if(opController.getPOV() == 0.0 ){
      if(!opPOVPressed){
        elbowPosition += 10;
      }
      opPOVPressed = true;
    }else if(opController.getPOV() == 180.0){
      if(!opPOVPressed){
        elbowPosition -= 10;
      }
      opPOVPressed = true;
    }else if(opController.getPOV() == 90.0 ){
      if(!opPOVPressed){
        elbowPosition += 2.5;
      }
      opPOVPressed = true;
    }else if(opController.getPOV() == 270.0){
      if(!opPOVPressed){
        elbowPosition -= 2.5;
      }
      opPOVPressed = true;
    }else{
      opPOVPressed = false;
    }

    elbowPosition = MathUtil.clamp(elbowPosition, 0, 100);

    if(opController.getAButton()){
      elbowPosition = 10;
    }else if(opController.getBButton()){
      elbowPosition = 77.5;
    }else if(opController.getYButtonPressed()){
      elbowPosition = 90;
    }else if(opController.getXButton()){
      elbowPosition = 80;
    }else if(opController.getRightBumperPressed()){
      elbowPosition = 0.0;
    }

    if(driveController.getYButtonPressed()){
      driveBase.resetGyro();
    }

    if(driveController.getBButton()){
      //driveBase.autoBalance();
    }else{
      driveBase.moveRobotFieldOriented(xVelocity, yVelocity, rotationVelocity);  //Meters per second & degrees per second
    }

    SmartDashboard.putNumber("X Velocity", xVelocity);
    SmartDashboard.putNumber("Y Velocity", yVelocity);
    SmartDashboard.putNumber("R Velocity", rotationVelocity);
    SmartDashboard.putNumber("Shoulder Power", shoulderPower);

    driveBase.printSwerveAngles();
    driveBase.printSwerveSpeeds();
    //driveBase.updatePIDLoops();

    arm.getShoulderPosition();
    arm.updateShoulderPID();

    intakeControl(opController);

    if(opController.getStartButtonPressed()){
      armOveride = !armOveride;
    }

    if(armOveride == false){
      arm.setShoulderPower(shoulderPower);
    }else{
      arm.setShoulderPosition(elbowPosition);
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

  public void intakeControl(XboxController controller) {
    double rightTrigger = controller.getRightTriggerAxis();
    double leftTrigger = controller.getLeftTriggerAxis();

    if(rightTrigger > 0.05) {
      intake.intakeObject();
    }else if(leftTrigger > 0.05){
      intake.outtakeObject();
    }else{
      intake.stopMotors();
    }
  }
}
