package frc.robot;

public class AutonSubstation2 extends Auton{

    public AutonSubstation2(DriveBase driveBase, Arm arm, Intake intake, boolean isBlue) {
        super(driveBase, arm, intake, isBlue);
        //TODO Auto-generated constructor stub
    }
    
    public void run(double time, double x){
        driveBase.autoBalance2();
    }
}
