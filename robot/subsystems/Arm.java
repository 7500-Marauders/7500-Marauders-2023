package frc.robot.subsystems;
//import code for robot util;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase{
    private CANSparkMax left = new CANSparkMax(Constants.ARM_LEFT, MotorType.kBrushless);
    private CANSparkMax right = new CANSparkMax(Constants.ARM_RIGHT, MotorType.kBrushless);
    private RelativeEncoder armEnc = left.getEncoder();
    //private SparkMaxAbsoluteEncoder lampEncoder = left.getAbsoluteEncoder(Type.kDutyCycle); /*new AnalogEncoder(null);*/
    public PIDController armController = new PIDController(Constants.ARM_kP, Constants.ARM_kI, Constants.ARM_kD);
    public ArmFeedforward armFeedforward = new ArmFeedforward(0, 0, 0,0);
    private DigitalInput armButton = new DigitalInput(0);

    int leftEncoderZero, rightEncoderZero;
    double[] angleList = {Constants.ARM_DOWN_ANGLE, Constants.ARM_MID_ANGLE, Constants.ARM_HIGH_ANGLE};
    boolean inManualMode = false;
    private double encZero;
    private double pwr =0;
    private boolean needToZero;
    

    public Arm(){
        left.setInverted(false);
        right.setInverted(false);
        needToZero = true;
        encZero =0;
        left.setIdleMode(IdleMode.kBrake);
        right.setIdleMode(IdleMode.kBrake);
    }

    public void periodic(){
        SmartDashboard.putNumber("Arm Power", pwr);
        SmartDashboard.putNumber("Arm Reading", getAngle());
        SmartDashboard.putBoolean("Arm in Manual", inManualMode);
        /*
        if(getLimitSwitch() && needToZero){
            resetEncoders();
            needToZero = false;
        }
        */
        SmartDashboard.putBoolean("Limit Switch", getLimitSwitch());
    }

    public void resetEncoders(){
        encZero = armEnc.getPosition();
    }
    
    public double getAngle(){
        return (armEnc.getPosition() - encZero) / Constants.ARM_GEAR_RATIO * Constants.WRIST_TICKS_PER_ROTATION /360.0 / 2.52;

    }

    // PreCondition: level = 0,1,2
    public void setArmAngle(double level){
        armController.setSetpoint(level);
    }
    public double getControllerOutput(){
        return MathUtil.clamp(armController.calculate(getAngle()), -0.3, 0.3); 
    }
    public boolean atLevel(){
        return armController.atSetpoint();
    }
    public void setArmPower(double power){
        pwr = power;
        left.set(pwr);    
    }
    public void turnToManual(){inManualMode = true;}
    public void turnToAuto(){inManualMode=false;}
    public boolean getManual(){
        return inManualMode;
    }
    public boolean getLimitSwitch(){
        return armButton.get();
    }
    public void brake(){
        left.setIdleMode(IdleMode.kBrake);
        right.setIdleMode(IdleMode.kBrake);
    }
}
 