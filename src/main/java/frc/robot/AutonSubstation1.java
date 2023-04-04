package frc.robot;

public class AutonSubstation1 extends Auton{

    public AutonSubstation1(DriveBase driveBase, Arm arm, Intake intake, boolean isBlue) {
        super(driveBase, arm, intake, isBlue);
        //TODO Auto-generated constructor stub
    }
    
    public void run(double time, double x, double a){
        double rotation;
        if(isBlue){
            rotation = -150;

            if(driveBase.getGyroAngle() < 182.5 && driveBase.getGyroAngle() > 177.5){
                rotation = 0;
            }else if(driveBase.getGyroAngle() < 190 && driveBase.getGyroAngle() > 180){
                rotation = -30;
            }else if(driveBase.getGyroAngle() > 170 && driveBase.getGyroAngle() < 180){
                rotation = 30;
            }else if(driveBase.getGyroAngle() < 170 && driveBase.getGyroAngle() > 90){
                rotation = 75;
            }
        }else{
            rotation = 150;
            if(driveBase.getGyroAngle() < 182.5 && driveBase.getGyroAngle() > 177.5){
                rotation = 0;
            }else if(driveBase.getGyroAngle() < 190 && driveBase.getGyroAngle() > 180){
                rotation = -30;
            }else if(driveBase.getGyroAngle() > 170 && driveBase.getGyroAngle() < 180){
                rotation = 30;
            }else if(driveBase.getGyroAngle() > 190 && driveBase.getGyroAngle() < 270){
                rotation = -75;
            }
        } 

        driveBase.moveRobotFieldOriented(0, 0, rotation);

        /* 
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
            if(isBlue){
                driveBase.moveRobotFieldOriented(-0.5, 0.0, 0.0);
            }else{
                driveBase.moveRobotFieldOriented(0.5, 0.0, 0.0);
            }
            arm.setShoulderPosition(0);
        }else if(driveBase.getGyroAngle() > 10 && driveBase.getGyroAngle() < 350 && time <14.0){
            if(isBlue){
                driveBase.moveRobotFieldOriented(0.1, 0.5, -100);
            }else{
                driveBase.moveRobotFieldOriented(-0.1, 0.5, 100);
            }
            arm.setShoulderPosition(0);
            intake.intakeObject();
        }else if(time <14.5){
            if(isBlue){
                driveBase.moveRobotFieldOriented(0.08, 0.5, 0.0);
            }else{
                driveBase.moveRobotFieldOriented(-0.08, 0.5, 0.0);
            }
            arm.setShoulderPosition(0);
            intake.intakeObject();
        }else{
            driveBase.moveRobotFieldOriented(0.0, 0.0, 0.0);
            arm.setShoulderPosition(0);
            intake.stopMotors();
        }*/
    }
}
