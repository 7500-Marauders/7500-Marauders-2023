package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class Balance extends CommandBase{
    private DriveTrain drive;
    private double lastPower;
    private double power;
    private double mult =1;
    private int count = 0;
    public Balance(DriveTrain drive){
        this.drive = drive;
        addRequirements(drive);
    }
    @Override
    public void initialize(){
        drive.brake();
    }
    @Override
    public void execute(){
        power = drive.getBalanceOutput();
        if((lastPower < 0 && power>0)||(power<0 && lastPower>0))
            count++;
        drive.arcadeDrive(power *mult * 0.28, 0);
        lastPower =power;
        if(drive.getPitch() < 4){
            mult = 0.7;}
        SmartDashboard.putNumber("oscillateCount", count);
        
          
    }
    @Override
    public void end(boolean interrupted){
        drive.arcadeDrive(0, 0);
    }
}