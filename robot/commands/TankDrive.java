package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class TankDrive extends CommandBase {
    private final DriveTrain drive;
    private DoubleSupplier leftY, rightX;

    public TankDrive(DriveTrain drive, DoubleSupplier leftY, DoubleSupplier rightX){
        this.drive = drive;
        this.leftY = leftY;
        this.rightX = rightX;
        addRequirements(drive);
    }

    @Override
    public void execute(){
        drive.set(leftY.getAsDouble(), rightX.getAsDouble());
        SmartDashboard.putNumber("leftY", leftY.getAsDouble());
        SmartDashboard.putNumber("rightX", rightX.getAsDouble());
    }

    @Override
    public void end(boolean interrupted){
        drive.set(0,0);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
