package frc.robot;

import javax.swing.LayoutFocusTraversalPolicy;

public class AutonFieldEdge2 extends Auton{
    double lastSavedTime;
    int i;
    boolean override = false;
    double xVelocity = 0.0;
    double yVelocity = 0.0;
    double rotation = 0.0;
    double armPosition = 0.0;
    int intakeState = 0;

    public AutonFieldEdge2(DriveBase driveBase, Arm arm, Intake intake, boolean isBlue) {
        super(driveBase, arm, intake, isBlue);
        //TODO Auto-generated constructor stub
        lastSavedTime = 0.0;
        i = 0;
    }
    
    public void run(double time, double x){
        override = false;
        xVelocity = 0.0;
        yVelocity = 0.0;
        rotation = 0.0;
        armPosition = 0.0;
        intakeState = 0;

        if(time > 14.5){
            i = 100;
        }

        if(i == 0){
            armPosition = 87.5;

            if(arm.getShoulderPosition() > 75){
                i++;
                lastSavedTime = time;
            }
        }else if(i == 1){
            yVelocity = -1.0;
            armPosition = 87.5;

            if(time - lastSavedTime > 0.75){
                i++;
                lastSavedTime = time;
            }
        }else if(i == 2){
            yVelocity = 0.5;
            armPosition = 87.5;
            intakeState = -1;

            if(time - lastSavedTime > 1.2){
                i++;
                lastSavedTime = time;
            }
        }else if(i == 3){
            armPosition = 0.0;

            if(arm.getShoulderPosition() < 30){
                i++;
                lastSavedTime = time;
            }
        }else if(i == 4){
            xVelocity = 0.75;
            rotation = -100.0;

            if(time - lastSavedTime > 0.5){
                i++;
                lastSavedTime = time;
            }
        }else if(i == 5){
            rotation = 100;
            if(driveBase.getGyroAngle() < 10 || driveBase.getGyroAngle() > 350){
                rotation = 10;
            }else if(driveBase.getGyroAngle() < 2.5 || driveBase.getGyroAngle() > 357.5){
                rotation = 0;
            }

            yVelocity = 2.0;

            if(time - lastSavedTime > 1.5){
                i++;
                lastSavedTime = time;
            }
        }else if(i == 6){
            yVelocity = 1.0;

            if(time - lastSavedTime > 0.5){
                i = 100;
                lastSavedTime = time;
            }
        }else if(i == 100){
            override = true;
            driveBase.stopWheels();
            arm.setShoulderPosition(arm.getShoulderPosition());
            intake.stopMotors();
        }

        if(!override){
            if(isBlue){
                driveBase.moveRobotFieldOriented(xVelocity, yVelocity, -rotation);
            }else{
                driveBase.moveRobotFieldOriented(-xVelocity, yVelocity, rotation);
            }
            arm.setShoulderPosition(armPosition);
            if(intakeState == -1){
                intake.outtakeObject();
            }else if(intakeState == 1){
                intake.intakeObject();
            }else{
                intake.stopMotors();
            }
        }
    }
}
