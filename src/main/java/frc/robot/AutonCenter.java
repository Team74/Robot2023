package frc.robot;

public class AutonCenter extends Auton{

    public AutonCenter(DriveBase driveBase, Arm arm, Intake intake) {
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
        }else if(time < 7.0){
            
            arm.setShoulderPosition(0);
        }else if(time < 9.0){
            
            driveBase.moveRobotFieldOriented(0.0, 0.85, 0.0);
            
            arm.setShoulderPosition(0);
        }else if(time < 14){
        
            driveBase.autoBalance2();
        }else{
            driveBase.lockWheels();
        }
    }
}
