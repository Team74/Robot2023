package frc.robot;

import javax.swing.LayoutFocusTraversalPolicy;

public class AutonFieldEdge2 extends Auton{
    double lastSavedTime;
    int i;
    double rotation;

    public AutonFieldEdge2(DriveBase driveBase, Arm arm, Intake intake, boolean isBlue) {
        super(driveBase, arm, intake, isBlue);
        //TODO Auto-generated constructor stub
        lastSavedTime = 0.0;
        i = 0;
    }
    
    public void run(double time, double x){
        if(time > 14.5){
            i = 100;
        }

        if(i == 0){
            driveBase.moveRobotFieldOriented(0, 0.0, 0);
            arm.setShoulderPosition(80);
            intake.stopMotors();

            if(arm.getShoulderPosition() > 70){
                i++;
                lastSavedTime = time;
            }
        }else if(i == 1){
            driveBase.moveRobotFieldOriented(0, -1.0, 0);
            arm.setShoulderPosition(80);
            intake.stopMotors();

            if(time - lastSavedTime > 0.5){
                i++;
                lastSavedTime = time;
            }
        }else if(i == 2){
            driveBase.moveRobotFieldOriented(0, 0.5, 0);
            arm.setShoulderPosition(75);
            intake.outtakeObject();

            if(time - lastSavedTime > 0.5){
                i++;
                lastSavedTime = time;
            }
        }else if(i == 3){
            if(isBlue){
                driveBase.moveRobotFieldOriented(-1.0, 0.0, -100);
            }else{
                driveBase.moveRobotFieldOriented(1.0, 0.0, 100);
            }
            arm.setShoulderPosition(0);
            intake.stopMotors();

            if(time - lastSavedTime > 0.5){
                i++;
                lastSavedTime = time;
            }
        }else if(i == 4){
            rotation = 100;
            if(driveBase.getGyroAngle() > 10 && driveBase.getGyroAngle() < 350){
                rotation = 10;
            }else if(driveBase.getGyroAngle() > 2.5 && driveBase.getGyroAngle() < 357.5){
                rotation = 10;
            }

            if(isBlue){
                driveBase.moveRobotFieldOriented(0.0, 2.0, -rotation);
            }else{
                driveBase.moveRobotFieldOriented(0.0, 2.0, rotation);
            }
            arm.setShoulderPosition(0);
            intake.stopMotors();

            if(time - lastSavedTime > 1.0){
                i++;
                lastSavedTime = time;
            }
        }else if(i == 100){
            driveBase.moveRobotFieldOriented(0, 0.0, 0);
            arm.setShoulderPosition(arm.getShoulderPosition());
            intake.stopMotors();
        }
    }
}
