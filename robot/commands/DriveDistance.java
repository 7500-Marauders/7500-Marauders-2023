package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class DriveDistance extends CommandBase{
    private DriveTrain drive;
    double distance, power;
    public DriveDistance(DriveTrain drive, double distance, double power){
        this.drive = drive;
        this.distance = distance;
        this.power = power;
        addRequirements(drive);
    }
    @Override
    public void initialize(){
        drive.resetEncoders();
        //drive.setDist(distance);
    }
    @Override
    public void execute(){
        drive.set(power); 
    }
    @Override
    public boolean isFinished(){
        return Math.abs(drive.getWheelAverage() - distance) < 5;
    }
    @Override
    public void end(boolean interrupted){
        drive.set(0);
    }
}
