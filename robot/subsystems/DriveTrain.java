package frc.robot.subsystems;


import org.opencv.core.Mat;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase{

    public boolean aControlMode = true;


    private final CANSparkMax leftMaster = new CANSparkMax(Constants.MOTOR_LEFT_BACK, MotorType.kBrushless);
    private final CANSparkMax leftSlave2 = new CANSparkMax(Constants.MOTOR_LEFT_MID, MotorType.kBrushless);
    private final CANSparkMax leftSlave = new CANSparkMax(Constants.MOTOR_LEFT_FRONT, MotorType.kBrushless);
    private final CANSparkMax rightMaster = new CANSparkMax(Constants.MOTOR_RIGHT_BACK, MotorType.kBrushless);
    private final CANSparkMax rightSlave2 = new CANSparkMax(Constants.MOTOR_RIGHT_MID, MotorType.kBrushless);
    private final CANSparkMax rightSlave = new CANSparkMax(Constants.MOTOR_RIGHT_FRONT, MotorType.kBrushless);

    CANSparkMax[] motors = {leftMaster, leftSlave, leftSlave2, rightMaster, rightSlave, rightSlave2};

    boolean arcadeOrTank = true;
    boolean lFlip = true;
    boolean rFlip = false;
    double yaw;
    double pitch;
    double roll;
    private final AHRS navimu = new AHRS(SPI.Port.kMXP);
    

    static double leftEncoderZero = 0;
    static double rightEncoderZero = 0;
    boolean inArcadeDrive = true;

    RelativeEncoder m_leftEncoder = leftSlave.getEncoder();
    RelativeEncoder m_rightEncoder = rightSlave.getEncoder();

    Thread front_cam;
    ShuffleboardTab PIDtab = Shuffleboard.getTab("PIDTab");
    public PIDController balancecontroller = new PIDController(Constants.BALANCE_kP, Constants.BALANCE_kI, Constants.BALANCE_kD);
    public PIDController turnController = new PIDController(Constants.TURN_kP, Constants.TURN_kI, Constants.TURN_kD);
    public PIDController forwardController = new PIDController(Constants.FORWARD_kP, Constants.FORWARD_kI, Constants.FORWARD_kD);
    private double multiplier;
    boolean signal;
    public double forwardTarget;
    double forward = 0, turn = 0;
    private double yawZero;

    //private final SlewRateLimiter throttleF;
    //private final SlewRateLimiter turnF;
    MotorControllerGroup rightGroup = new MotorControllerGroup(rightSlave, rightSlave2, rightMaster);
    MotorControllerGroup leftGroup = new MotorControllerGroup(leftSlave, leftSlave2, leftMaster);
    
    //DifferentialDrive drive = new DifferentialDrive(leftGroup, rightGroup);

    //NetworkTable table = NetworkTableInstance.getDefault().getTable()

    public DriveTrain(){

        leftMaster.setInverted(lFlip);
        leftSlave.setInverted(lFlip);
        leftSlave2.setInverted(lFlip);
        rightMaster.setInverted(rFlip);
        rightSlave.setInverted(rFlip);
        rightSlave2.setInverted(rFlip);
        leftMaster.setIdleMode(IdleMode.kCoast);
        leftSlave.setIdleMode(IdleMode.kCoast);
        leftSlave2.setIdleMode(IdleMode.kCoast);
        rightMaster.setIdleMode(IdleMode.kCoast);
        rightSlave.setIdleMode(IdleMode.kCoast);
        rightSlave2.setIdleMode(IdleMode.kCoast);
        

        navimu.calibrate();
        PIDtab.add("PID", balancecontroller).withWidget(BuiltInWidgets.kPIDController);
        PIDtab.add("Turn", turnController).withWidget(BuiltInWidgets.kPIDController);
        PIDtab.add("Forward", forwardController).withWidget(BuiltInWidgets.kPIDController);
        SmartDashboard.putData("Scheduler", CommandScheduler.getInstance());
        
        turnController.enableContinuousInput(-180, 180); //delete if turn controller buggin
        
        front_cam = new Thread(()-> {UsbCamera frontCam = CameraServer.startAutomaticCapture(0);
        
        frontCam.setResolution(240, 135);
        

        CvSink cvSink = CameraServer.getVideo();
        CvSource outputStream = CameraServer.putVideo("Front Cam", 240, 135);

        Mat mat = new Mat();

        while(!Thread.interrupted()) {
            if(cvSink.grabFrame(mat)==0){
                outputStream.notifyError(cvSink.getError());
                continue;
            }   
        }
        }
        );

        front_cam.setDaemon(true);
        front_cam.start();
        multiplier = 0.60;
        balancecontroller.setTolerance(3);
        turnController.setTolerance(1);
        forwardController.setTolerance(.05);
        resetEncoders();   

        //throttleF = new SlewRateLimiter(24);
        //turnF = new SlewRateLimiter(24);
        yawZero = 0;
    }


    public void periodic(){
        yaw = -navimu.getYaw() - yawZero;
        pitch = navimu.getRoll(); 
        roll = navimu.getPitch();
        SmartDashboard.putNumber("pitch",pitch);
        SmartDashboard.putNumber("yaw", yaw);
        SmartDashboard.putNumber("roll", roll);
        SmartDashboard.putNumber("encoderL", getLeftDistanceTraveled());
        SmartDashboard.putNumber("encoderR", getRightDistanceTraveled());
        SmartDashboard.putNumber("wheel Average", getWheelAverage());
        SmartDashboard.putBoolean("gyro on",!navimu.isCalibrating());
        SmartDashboard.putNumber("ForwardTarget", forwardController.getSetpoint());
        SmartDashboard.putBoolean("AtForward?", forwardAtSetpoint());
        SmartDashboard.putNumber("forwardOutput", getForwardOutput());
        SmartDashboard.putBoolean("aControlMode", aControlMode);
    }

    // public void arcadeDrivee(double forward, double turn){
    //     //double driveOutput = throttleF.calculate(forward);
    //     //double turnOutput = turnF.calculate(turn);
        
    //     drive.arcadeDrive(throttleF.calculate(forward)*multiplier, -turnF.calculate(turn)*multiplier);
    //     // leftMaster.set((turn + forward)*multiplier);
    //     // leftSlave.set((turn +forward)*multiplier);
    //     // rightMaster.set((-turn + forward)*multiplier);
    //     // rightSlave.set((-turn+ forward)*multiplier);
    //     // rightSlave2.set((-turn + forward)*multiplier);
    //     // leftSlave2.set((turn+forward)*multiplier);
    // }

    public void arcadeDrive(double forward, double turn){
        leftGroup.set((forward + turn)*multiplier );
        rightGroup.set((forward-turn)*multiplier );
    }

    public void set(double speed){
        leftMaster.set(speed);
        leftSlave.set(speed);
        leftSlave2.set(speed);
        rightMaster.set(speed);
        rightSlave.set(speed);
        rightSlave2.set(speed);
    }


    public void set(double lSpeed, double rSpeed){
        leftMaster.set(lSpeed);
        leftSlave.set(lSpeed);
        leftSlave2.set(lSpeed);
        rightMaster.set(rSpeed);
        rightSlave.set(rSpeed);
        rightSlave2.set(rSpeed);
    }

    public boolean getControlMehtod(){return inArcadeDrive;}

    public void setDist(double dist){
        forwardTarget = dist;
    
    }

    public boolean atDist(){
        return signal;
    }
   
    public double getLeftDistanceTraveled(){
        return (getLeftEncoderPos() * (Units.inchesToMeters(Constants.DRIVE_WHEEL_RADIUS) * 2 * Math.PI));
    }

    public double getRightDistanceTraveled(){
        return (getRightEncoderPos() *(Units.inchesToMeters(Constants.DRIVE_WHEEL_RADIUS) * 2 * Math.PI));
    }

    public double getLeftEncoderPos(){
        return ((m_leftEncoder.getPosition() - leftEncoderZero)) / Constants.DRIVE_GEAR_RATIO;
    }

    public double getRightEncoderPos(){
        return ((m_rightEncoder.getPosition() - rightEncoderZero)) / Constants.DRIVE_GEAR_RATIO;
    }
    
    public double getWheelAverage(){
        return (getLeftDistanceTraveled() + getRightDistanceTraveled())/2;
    }

    public void resetEncoders(){
        leftEncoderZero = m_leftEncoder.getPosition();
        rightEncoderZero = m_rightEncoder.getPosition();
    }

    public boolean getArcadeOrTank(){
        return arcadeOrTank;
    }
    public void switchControl(){
        arcadeOrTank = !arcadeOrTank;
    }
    public Command switchControlMethod(){
        return Commands.runOnce(this::switchControlMethod, this);
    }
    public double getYaw(){
        return yaw;
    }
    public double getPitch(){
        return pitch;
    }
    public double getRoll(){
        return roll;
    }
    public double getBalanceOutput(){
        return  MathUtil.clamp(balancecontroller.calculate(pitch, 0), -0.8,0.8);
    }

    public double getTurnOutput(){
        return MathUtil.clamp((turnController.calculate(yaw)),-.5,.5);
    }
    public double getForwardOutput(){
        return forwardController.calculate(getWheelAverage(), forwardTarget);
    }
    public boolean forwardAtSetpoint(){
        return forwardController.atSetpoint();
    }
    public void setMultiplier(double m){
        multiplier = m;
    }
    public boolean botOnRamp(){
        return Math.abs(pitch) > 7;
    }
    public void setTargetAngle(double angle){
        turnController.setSetpoint(angle);
    }
    public void calibrateGyro(){
        navimu.calibrate();
    }
    public void resetYaw(){
        yawZero = yaw;
    }

    public void brake(){
        for(CANSparkMax m : motors){
            m.setIdleMode(IdleMode.kBrake);
        }
    }

    public void coast(){
        for(CANSparkMax m: motors){
            m.setIdleMode(IdleMode.kCoast);
        }
    }
}
