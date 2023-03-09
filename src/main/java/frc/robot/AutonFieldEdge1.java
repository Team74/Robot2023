package frc.robot;

public class AutonFieldEdge1 extends Auton{

    public AutonFieldEdge1(DriveBase driveBase, Arm arm, Intake intake) {
        super(driveBase, arm, intake);
        //TODO Auto-generated constructor stub
    }
    
    public void run(double time, double x){

        if(time < 2.0){
            arm.setShoulderPosition(85);
        }else if(time < 3.5){
            driveBase.moveRobotFieldOriented(0.0, -0.5, 0.0);
            arm.setShoulderPosition(85);
        }else if(time < 4.5){
            driveBase.moveRobotFieldOriented(0.0, 0.0, 0.0);
            arm.setShoulderPosition(85);
            intake.outtakeObject();
        }else if(time < 6.0){
            driveBase.moveRobotFieldOriented(0.0, 0.5, 0.0);
            arm.setShoulderPosition(85);
            intake.stopMotors();
        }else if(time <7.0){
            driveBase.moveRobotFieldOriented(-0.5, 0.0, 0.0);
            arm.setShoulderPosition(0);
        }else if(driveBase.getGyroAngle() > 10 && driveBase.getGyroAngle() < 350 && time <14.0){
            driveBase.moveRobotFieldOriented(0.1, 0.5, -100);
            arm.setShoulderPosition(0);
            intake.intakeObject();
        }else if(time <14.0){
            driveBase.moveRobotFieldOriented(0.08, 0.5, 0.0);
            arm.setShoulderPosition(0);
            intake.intakeObject();
        }else{
            driveBase.moveRobotFieldOriented(0.0, 0.0, 0.0);
            arm.setShoulderPosition(0);
            intake.stopMotors();
        }

    }
}
