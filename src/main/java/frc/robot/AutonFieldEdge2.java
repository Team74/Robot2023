package frc.robot;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
    
    public void run(double time, double x, double a){
        NetworkTableEntry pipeline = table.getEntry("pipeline");
        SmartDashboard.putBoolean("Is Blue?", isBlue);
        
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
            armPosition = 92.5;

            if(time > 0.7){
                i++;
                lastSavedTime = time;
            }
        }else if(i == 1){
            yVelocity = -1.0;
            armPosition = 92.5;

            if(time - lastSavedTime > 0.90){
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

            if(time - lastSavedTime > 0.4){
                i++;
                lastSavedTime = time;
            }
        }else if(i == 4){
            xVelocity = 0.75;
            rotation = -120.0;

            if(time - lastSavedTime > 0.5){
                i++;
                lastSavedTime = time;
            }
        }else if(i == 5){
            if(isBlue){
                rotation = 100;
    
                if(driveBase.getGyroAngle() < 2.5 || driveBase.getGyroAngle() > 357.5){
                    rotation = 0;
                }else if(driveBase.getGyroAngle() < 10){
                    rotation = 30;
                }else if(driveBase.getGyroAngle() > 350){
                    rotation = -30;
                }else if(driveBase.getGyroAngle() > 90){
                    rotation = -100;
                }
            }else{
                rotation = 100;

                if(driveBase.getGyroAngle() < 2.5 || driveBase.getGyroAngle() > 357.5){
                    rotation = 0;
                }else if(driveBase.getGyroAngle() < 10){
                    rotation = -30;
                }else if(driveBase.getGyroAngle() > 350){
                    rotation = 30;
                }else if(driveBase.getGyroAngle() < 270){
                    rotation = -100;
                }
            } 

            xVelocity = -0.325;
            yVelocity = 2.0;

            if(time - lastSavedTime > 1.5){
                i++;
                lastSavedTime = time;
                pipeline.setNumber(1);
            }
        }else if(i == 6){
            override = true;
            driveBase.moveRobotFieldOriented(x * 0.020, 1.0, 0.0);
            arm.setShoulderPosition(0.0);
            intake.intakeObject();

            if(intake.intakeLimit()){
                i++;
                lastSavedTime = time;
                pipeline.setNumber(0);
            }
            else if(time - lastSavedTime > 2.5){
                i = 100;
                lastSavedTime = time;
                pipeline.setNumber(0);
            }
        }else if(i == 7){
            rotation = 150;
            if(isBlue){
                rotation = 150;
    
                if(driveBase.getGyroAngle() < 182.5 && driveBase.getGyroAngle() > 177.5){
                    rotation = 0;
                }else if(driveBase.getGyroAngle() < 190 && driveBase.getGyroAngle() > 180){
                    rotation = 30;
                }else if(driveBase.getGyroAngle() > 170 && driveBase.getGyroAngle() < 180){
                    rotation = -30;
                }else if(driveBase.getGyroAngle() < 170 && driveBase.getGyroAngle() > 90){
                    rotation = -75;
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
            xVelocity = -0.290;
            yVelocity = -2;
            armPosition = 15;

            if(time - lastSavedTime > 2.0){
                i++;
                lastSavedTime = time;
            }   
        }else if(i == 8){
            override = true;
            driveBase.moveRobotFieldOriented(0.1, -1.00, 0.0);
            arm.setShoulderPosition(97.5);
            intake.stopMotors();

            if(time - lastSavedTime > 1.0){
                i++;
                lastSavedTime = time;
            } 
        }else if(i == 9){
            override = true;
            driveBase.moveRobotFieldOriented(0.0, -1.0, 0.0);
            arm.setShoulderPosition(97.5);
            intake.stopMotors();

            if(time - lastSavedTime > 0.85){
                i++;
                lastSavedTime = time;
            } 
        }else if(i == 10){
            override = true;
            double speed = 0;
            if(a < .150){
                speed = -0.3;
            }else if(a > 0.170){
                speed = 0.2;
            }

            driveBase.moveRobotFieldOriented(0.1, speed, (x - 5) * 2);
            arm.setShoulderPosition(97.5);
            intake.stopMotors();

            if(time - lastSavedTime > 1.5){
                i++;
                lastSavedTime = time;
            } 
        }else if(i == 11){
            armPosition = 82.5;

            if(time - lastSavedTime > 0.2){
                i++;
                lastSavedTime = time;
            }
        }else if(i == 12){
            armPosition = 82.5;
            intakeState = -1;
            yVelocity = 0.5;

            if(time - lastSavedTime > 1.0){
                i = 100;
                lastSavedTime = time;
            }
        }else if(i > 99){
            override = true;
            driveBase.stopWheels();
            arm.setShoulderPosition(arm.getShoulderPosition());
            intake.stopMotors();
        }

        //x = 5
        //a = 2.5

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
