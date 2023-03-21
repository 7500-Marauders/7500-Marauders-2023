package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class ArcadeDrive extends CommandBase{
    private DriveTrain driveTrain;
    private DoubleSupplier left, rightX, rightY;
    public ArcadeDrive(DriveTrain driveTrain, DoubleSupplier left, DoubleSupplier rightX, DoubleSupplier rightY){
        this.driveTrain = driveTrain;
        this.left = left;
        this.rightY =rightY;
        this.rightX = rightX;
        addRequirements(driveTrain);
    }
    @Override
    public void initialize(){
        driveTrain.coast();
    }
    @Override
    public void execute(){
         if(driveTrain.getControlMehtod()){
            if(Math.abs(left.getAsDouble()) > 0.1 || Math.abs(rightX.getAsDouble()) > 0.17)
                driveTrain.arcadeDrive(-left.getAsDouble(), rightX.getAsDouble());
            else
                driveTrain.arcadeDrive(0, 0);
        }
        else driveTrain.set(left.getAsDouble(), rightY.getAsDouble());
    
    }
    
}
