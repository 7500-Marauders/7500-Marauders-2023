package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class ManualArm extends CommandBase{
    
    Arm arm;
    DoubleSupplier forwardPwr;
    //BooleanSupplier inManual;
    public ManualArm(Arm arm, DoubleSupplier forwardPwr){//, BooleanSupplier inManual){
        this.arm = arm;
        this.forwardPwr = forwardPwr;
        //this.inManual = inManual;
        addRequirements(arm);
       
    }

    @Override
    public void execute(){
        //if(inManual.getAsBoolean())
            arm.setArmPower((forwardPwr.getAsDouble())*0.25);
        //else
            //arm.setArmPower(0);
    }
            

    @Override
    public void end(boolean interrupted){
        arm.setArmPower(0);
    }
}
