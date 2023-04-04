package frc.robot;

public class AutonCenter extends Auton{

    public AutonCenter(DriveBase driveBase, Arm arm, Intake intake, boolean isBlue) {
        super(driveBase, arm, intake, isBlue);
        //TODO Auto-generated constructor stub
    }
    
    public void run(double time, double x, double a){

        if(time < 0.9){
            driveBase.moveRobotFieldOriented(0.0, 0.0, 0.0);
            arm.setShoulderPosition(92.5);
        }else if(time < 1.8){
            driveBase.moveRobotFieldOriented(0.0, -1.0, 0.0);
            arm.setShoulderPosition(92.5);
        }else if(time < 3.0){
            driveBase.moveRobotFieldOriented(0.0, 0.5, 0.0);
            arm.setShoulderPosition(87.5);
            intake.outtakeObject();
        }else if(time < 4.2){
            driveBase.moveRobotFieldOriented(0.0, 0.5, 0.0);
            arm.setShoulderPosition(0.0);
            intake.stopMotors();
        }else if(time < 6.4){
            driveBase.moveRobotFieldOriented(0.0, 0.9, 0.0);
            arm.setShoulderPosition(0);
        }else if(time < 7.0){
            driveBase.lockWheels();
        }else if(time < 14.9){
        
            driveBase.autoBalance2();
        }else{
            driveBase.lockWheels();
        }
    }
}
