package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Wrist extends SubsystemBase {
    private final CANSparkMax wrist = new CANSparkMax(Constants.WRIST, MotorType.kBrushless);
    private PIDController angleController = new PIDController(.0075, 0, 0);
    private double encZero;
    private RelativeEncoder enc = wrist.getEncoder();
    public double[] coneScoreLevel = {Constants.WRIST_DOWN_CONE, Constants.WRIST_MID_CONE, Constants.WRIST_HIGH_CONE};
    public double[] cubeScoreLevel = {Constants.WRIST_DOWN_CUBE, Constants.WRIST_MID_CUBE, Constants.WRIST_HIGH_CONE};
    private boolean inManual;
    ShuffleboardTab PIDtab = Shuffleboard.getTab("PIDTab");
    

    public Wrist(){
        encZero = 0;
        enc.setPosition(encZero);
        wrist.setIdleMode(IdleMode.kBrake);
        inManual = false;
        
        PIDtab.add("Wrist", angleController).withWidget(BuiltInWidgets.kPIDController);
    
    }

    
    public void periodic(){
        SmartDashboard.putNumber("Wrist Pos", getAngle());
    
    }

    public double getAngle(){
        return (enc.getPosition() - encZero) / Constants.WRIST_GEAR_RATIO * Constants.WRIST_TICKS_PER_ROTATION /360.0;
    }

    public void setTarget(double level){
        angleController.setSetpoint(level);
    }

    public double getControllerOutput(){
        return MathUtil.clamp(angleController.calculate(getAngle()), -.3, .3);
    }

    public boolean controllerAtSetPoint(){
        return angleController.atSetpoint();
    }

    public void set(double pwr){
        wrist.set(pwr);
    }

    public void manualSet(DoubleSupplier pwr){
        wrist.set(-pwr.getAsDouble() * 0.15);
    }

    public void zeroEncoder(){
        encZero = enc.getPosition();
    }

    public void toAuto(){
        inManual =false;
    }

    public void toManual(){
        inManual = true;
    }

    public boolean inManualMode(){
        return inManual;
    }
    public void brake(){
        wrist.setIdleMode(IdleMode.kBrake);
    }
}
