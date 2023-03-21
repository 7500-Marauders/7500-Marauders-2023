package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Claw extends SubsystemBase {
    private final CANSparkMax claw = new CANSparkMax(Constants.CLAW, MotorType.kBrushless);
    private final RelativeEncoder enc = claw.getEncoder();
    private double encZero;

    public Claw(){
        encZero =0;
        claw.setIdleMode(IdleMode.kCoast);
        SmartDashboard.putNumber("claw position", getPosition());
        SmartDashboard.putNumber("voltage", claw.getBusVoltage());
    }
    @Override
    public void periodic(){
        SmartDashboard.putNumber("Nominal Volt", claw.getVoltageCompensationNominalVoltage());
        SmartDashboard.putNumber("get", claw.get());
        SmartDashboard.putNumber("Applied Out", claw.getAppliedOutput());
        SmartDashboard.putNumber("Out Current", claw.getOutputCurrent());
        SmartDashboard.putBoolean("boolean", claw.getOutputCurrent() > 60);

    }

    public void setPower(DoubleSupplier rPwr, DoubleSupplier lPwr){
        claw.set(rPwr.getAsDouble()*0.582 - lPwr.getAsDouble()*0.25);
    }

    public double getOutCurrent(){
        return claw.getOutputCurrent();
    }

    public void setPowerAuto(double pwr){
        claw.set(pwr);
    }

    public void repelIntake(){
        claw.set(-1);
    }

    public double getPosition(){
        return (enc.getPosition() - encZero) / Constants.CLAW_GEAR_RATIO * Constants.CLAW_TICKS_PER_ROTATION;
    }

    public void zeroEncoder(){
        encZero = enc.getPosition();
        
    }

   
    
}
