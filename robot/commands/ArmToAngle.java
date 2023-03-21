package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class ArmToAngle extends CommandBase{
    
    Arm arm;
    double level;
    public ArmToAngle(Arm arm, double level){
        this.arm = arm;
        this.level = level;
        addRequirements(arm);
    }

    @Override
    public void initialize(){
        arm.setArmAngle(level);
    }
    @Override
    public void execute(){
        arm.setArmPower(arm.getControllerOutput());
    }
   

    @Override
    public void end(boolean interrupted){
        arm.setArmPower(0);
    }
    
}
