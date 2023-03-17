package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Auton {
    protected DriveBase driveBase;
    protected Arm arm;
    protected Intake intake;
    protected NetworkTable table;
    protected boolean isBlue;

    public Auton(DriveBase driveBase, Arm arm, Intake intake, boolean isBlue){
        this.driveBase = driveBase;
        this.arm = arm;
        this.intake = intake;
        this.isBlue = isBlue;

        table = NetworkTableInstance.getDefault().getTable("limelight");

    }

    public void run(double time, double x){
    }
}
