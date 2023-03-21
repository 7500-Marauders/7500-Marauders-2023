package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Claw;

public class Suck extends CommandBase{
    private Claw claw;
    public Suck(Claw claw){
        this.claw = claw;
    }

    @Override
    public void execute(){
        claw.setPowerAuto(Constants.DAVIS_CONSTANT);
    }

    /*@Override
    public boolean isFinished(){
        return 
    }*/
}
