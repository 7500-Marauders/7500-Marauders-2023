package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class AutoAlign extends CommandBase{
    DriveTrain drive;
    double angle;
    public AutoAlign(DriveTrain drive, double angle){
        this.drive = drive;
        this.angle = angle;
        addRequirements(drive);
    }
    @Override
    public void initialize(){
        drive.setTargetAngle(angle);
    }
    @Override
    public void execute(){
        drive.arcadeDrive(0, (-drive.getTurnOutput()));
    }
    
    @Override
    public void end(boolean interrupted){
        drive.arcadeDrive(0, 0);
    }
}
