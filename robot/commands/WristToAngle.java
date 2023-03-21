package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Wrist;

public class WristToAngle extends CommandBase{
    
    private Wrist wrist;
    private double level;
    
    public WristToAngle(Wrist wrist, double level){
        this.level = level;
        this.wrist = wrist;
        addRequirements(wrist);
    }

    @Override
    public void initialize(){
        wrist.setTarget(level);
    }

    @Override
    public void execute(){
        wrist.set(wrist.getControllerOutput());
    }

    @Override
    public void end(boolean interrupted){
        wrist.set(0);
    }
    
}
