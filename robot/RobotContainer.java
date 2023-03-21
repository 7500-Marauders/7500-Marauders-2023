// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.ArmToAngle;
import frc.robot.commands.AutoAlign;
import frc.robot.commands.Balance;
import frc.robot.commands.DriveDistance;
import frc.robot.commands.ManualArm;
import frc.robot.commands.TankDrive;
import frc.robot.commands.WristToAngle;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Wrist;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private DriveTrain drive;
  private Arm arm;
  private Claw claw;
  private Wrist wrist;
  private Command arcadeDrive, tankDrive;
  private Command driveAlign /* , armAlign*/;
  private Command calibrateGyro, resetYaw;
  private Trigger arcade, tank, caliGyroTrig;
  private Trigger driveAlignButton /*,armAlignButton*/;
  private CommandBase slow, fast, vFast, vvFast;
  private Trigger holdForSlowDrive, holdForVFastDrive, holdForVVFastDrive;
  private Command armDefault, armConeMid;
  private Command manualWristForward, manualWristBackwards, manualWristStop, manualClaw, powerArm;
  private Trigger manualWristForwardTrig, manualWristBackwardsTrig, coneIntakeTrigger, cubeIntakeTrigger, repelClawConeTrigger, repelClawCubeTrigger;
  private Command wristIntakeCube, wristIntakeCone, armIntakeCone, armIntakeCube;
  private ParallelCommandGroup intakeCube, intakeCone;
  private Command clawIntakeCone, clawIntakeCube, repelClawCone, repelClawCube, stopIntake;
  private ParallelCommandGroup  scoreCubeMid, scoreConeMid;
  private Command wristConeScoreLow, wristConeScoreMid, wristConeScoreHigh, wristDefault, secureCone;
  private Command wristCubeScoreLow, wristCubeScoreMid, armCubeMid;
  private Trigger coneScoreLowTrig, coneScoreTrig, cubeScoreLowTrig, cubeScoreTrig;
  private Command expelCone, expelCube, mechDefault;
  private Command manualMode, autoMode;
  private Trigger manualModeTrig, autoModeTrig;
  private Command mechDefault2, wristDefault2;
  private ParallelCommandGroup scoreCubeMid2;
  private ParallelCommandGroup scoreConeMid2;
  private WristToAngle wristConeScoreMid2;
  private WristToAngle wristCubeScoreMid2;
  private ArmToAngle armConeMid2;
  private ArmToAngle armCubeMid2;
  private Trigger humanPlayerConeTrig, humanPlayerCubeTrig;
  private Command intakeHP, humanPlayerCube;
  private Command humanToSecureArm, humanToSecureWrist;
  private Trigger humanSecureTrig;
  private final CommandJoystick leftJoystick, rightJoystick;
  private final CommandXboxController mechController;
  private WristToAngle wristIntakeHP;
  private RunCommand clawIntakeConeHP;
  private RunCommand clawHumanPlayerCube;
  private ArmToAngle armIntakeHP;
  private ArmToAngle armHumanPlayerCube;
  private WristToAngle wristDefault3;
  private ParallelCommandGroup mechDefault3;
  private Trigger humanPlayerIntakeTrig;
  private WristToAngle wristIntakeHP2;
  private Command bal;
  private Trigger balanceTrig;
  private Trigger mControlModeTrig, aControlModeTrig;
  private Command mControlMode, aControlMode, powerWrist;

 
 
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    leftJoystick = new CommandJoystick(0);
    rightJoystick = new CommandJoystick(1);
    mechController = new CommandXboxController(2);

    //initialize and register subsystem
    drive = new DriveTrain();
    drive.register();
    arm = new Arm();
    arm.register();
    claw = new Claw();
    claw.register();
    wrist = new Wrist();
    wrist.register();

    //Drive Train commands
    arcadeDrive = new ArcadeDrive(drive, leftJoystick::getY, rightJoystick::getX, rightJoystick::getY);
    tankDrive = new TankDrive(drive, leftJoystick::getY, rightJoystick::getY);
    bal = new Balance(drive);
    driveAlign = new AutoAlign(drive,180);
    slow = new InstantCommand(()->drive.setMultiplier(0.20));
    fast = new InstantCommand(()->drive.setMultiplier(0.60));
    vFast = new InstantCommand(()->drive.setMultiplier(0.80));
    vvFast = new InstantCommand(()->drive.setMultiplier(1));
    calibrateGyro = new InstantCommand(drive::calibrateGyro);
  
    //Arm Commands
    powerArm = new ManualArm(arm, mechController::getLeftY);
    armIntakeCone = new ArmToAngle(arm, -15);
    armIntakeCube = new ArmToAngle(arm, -12);
    armIntakeHP = new ArmToAngle(arm, -107);
    armDefault = new ArmToAngle(arm, -5);
    armConeMid = new ArmToAngle(arm, -79.66);
    armConeMid2 = new ArmToAngle(arm, -79.66);
    armCubeMid = new ArmToAngle(arm, -58);
    armCubeMid2 = new ArmToAngle(arm, -58);
    humanToSecureArm = new ArmToAngle(arm, -100);
    
    //Claw commands 
    manualClaw = new RunCommand(()->claw.setPower(mechController::getRightTriggerAxis, mechController::getLeftTriggerAxis),claw);
    clawIntakeCone = new RunCommand(()->claw.setPowerAuto(0.6), claw);
    clawIntakeCube = new RunCommand(()->claw.setPowerAuto(-0.282), claw);
    clawIntakeConeHP = new RunCommand(()->claw.setPowerAuto(0.3), claw);
    stopIntake = new RunCommand(()->claw.setPowerAuto(0.15), claw);
    repelClawCone = new RunCommand(()->claw.setPowerAuto(-1), claw);
    repelClawCube = new RunCommand(()->claw.setPowerAuto(1), claw);
   
    //wrist
    wristIntakeCone = new WristToAngle(wrist, 87);
    wristIntakeCube = new WristToAngle(wrist, 81);
    wristIntakeHP = new WristToAngle(wrist, 150);
    wristIntakeHP2 = new WristToAngle(wrist, 150);
    wristDefault = new WristToAngle(wrist, 30);
    secureCone = new WristToAngle(wrist, 30);

    wristConeScoreLow = new WristToAngle(wrist, wrist.coneScoreLevel[0]);
    wristConeScoreMid = new WristToAngle(wrist, wrist.coneScoreLevel[1]);
    wristConeScoreMid2 = new WristToAngle(wrist, wrist.coneScoreLevel[1]);

    wristCubeScoreLow = new WristToAngle(wrist, wrist.cubeScoreLevel[0]);
    wristCubeScoreMid = new WristToAngle(wrist, wrist.cubeScoreLevel[1]);
    wristCubeScoreMid2 = new WristToAngle(wrist, wrist.cubeScoreLevel[1]);

    manualWristForward = new RunCommand(()->wrist.set(0.2), wrist);
    manualWristBackwards = new RunCommand(()->wrist.set(-0.2), wrist);
    manualWristStop = new RunCommand(()->wrist.set(0),wrist);

    powerWrist = new RunCommand(()->wrist.manualSet(mechController::getRightY), wrist);
    
    //Groups
    //intakeCone = new ParallelCommandGroup(wristIntakeCone, armIntakeCone, clawIntakeCone);
    intakeCube = new ParallelCommandGroup(wristIntakeCube, armIntakeCube, clawIntakeCube);
    intakeHP = new ParallelCommandGroup(wristIntakeHP, armIntakeHP, clawIntakeConeHP);
    mechDefault = new ParallelCommandGroup(armDefault, wristDefault);

    expelCone = new SequentialCommandGroup(
      new RunCommand(()->claw.setPowerAuto(-1), claw).withTimeout(1), 
      new RunCommand(()->claw.setPowerAuto(0), claw));

    expelCube = new SequentialCommandGroup(
      new RunCommand(()->claw.setPowerAuto(0.4), claw).withTimeout(0.4), 
      new InstantCommand(()->claw.setPowerAuto(0), claw));

    scoreConeMid = new ParallelCommandGroup(wristConeScoreMid, armConeMid);
    scoreConeMid2 = new ParallelCommandGroup(wristConeScoreMid2, armConeMid2);

    scoreCubeMid = new ParallelCommandGroup(wristCubeScoreMid, armCubeMid);
    scoreCubeMid2 = new ParallelCommandGroup(wristCubeScoreMid2, armCubeMid2);
  

    configureBindings();
    
    //Drive Train triggers
    balanceTrig.whileTrue(bal).onFalse(arcadeDrive);
    holdForSlowDrive.whileTrue(slow).whileFalse(fast);
    holdForVFastDrive.whileTrue(vFast).whileFalse(fast);
    holdForVVFastDrive.whileTrue(vvFast).whileFalse(fast);
    caliGyroTrig.onTrue(calibrateGyro);
    arcade.onTrue(arcadeDrive);
    tank.onTrue(tankDrive);
    manualWristBackwardsTrig.whileTrue(manualWristBackwards).onFalse(manualWristStop);
    manualWristForwardTrig.whileTrue(manualWristForward).onFalse(manualWristStop);

    cubeIntakeTrigger
      .onTrue(intakeCube.until(()->claw.getOutCurrent() > 60))
      .onFalse(new RunCommand(()->claw.setPowerAuto(-0.12), claw))
      .onFalse(new WristToAngle(wrist, 35))
      .onFalse(new ArmToAngle(arm, -30));

    coneIntakeTrigger
      .whileTrue(wristIntakeCone)
      .whileTrue(armIntakeCone)
      .onTrue(clawIntakeCone.until(()->claw.getOutCurrent() > 60).andThen(Commands.waitSeconds(0.6))
        .andThen(new RunCommand(()->claw.setPowerAuto(0.6))
        .until(()->claw.getOutCurrent()>60)).andThen(
        new InstantCommand(()->claw.setPowerAuto(0.15), claw)))
      .onFalse(manualWristStop)
      .onFalse(stopIntake)
      .whileFalse(secureCone);

    cubeScoreTrig
      .onTrue(scoreCubeMid)
      .onFalse(expelCube.withTimeout(0.5))
      .onFalse(new SequentialCommandGroup(scoreCubeMid2.withTimeout(1.5), secureCone()));

    coneScoreTrig.whileTrue(scoreConeMid)
      .onFalse(expelCone.withTimeout(0.4).andThen(new RunCommand(()->claw.setPowerAuto(0), claw)))
      .onFalse(new SequentialCommandGroup(scoreConeMid2.withTimeout(1.5), Commands.waitSeconds(1.5), secureCone()));

    repelClawConeTrigger.whileTrue(repelClawCone).onFalse(manualWristStop).onFalse(stopIntake);
    repelClawCubeTrigger.whileTrue(repelClawCube).onFalse(manualWristStop).onFalse(stopIntake);

    humanPlayerConeTrig
      .onTrue(intakeHP)
      .onFalse(new ArmToAngle(arm, -86))
      .onFalse(new RunCommand(()->claw.setPowerAuto(0.2), claw));

    humanSecureTrig.onTrue(humanToSecureArm).onFalse(secureCone()).onFalse(new RunCommand(()->claw.setPowerAuto(0.2), claw));
    humanPlayerIntakeTrig
      .onTrue(humanToSecureArm)
      .onTrue(wristIntakeHP2)
      .onTrue(new RunCommand(()->claw.setPowerAuto(0.45), claw))
      .onFalse(secureCone())
      .onFalse(new RunCommand(()->claw.setPowerAuto(0.15), claw));
    CommandScheduler.getInstance().schedule(arcadeDrive);

    mControlModeTrig.onTrue(powerArm).onTrue(powerWrist).onTrue(manualClaw);
    
    drive.setDefaultCommand(arcadeDrive);
    
  }

  //Bind triggers to physical buttons on joysticks and controllers
  private void configureBindings() {

    
    arcade = rightJoystick.button(3);
    tank = rightJoystick.button(4);
    balanceTrig = mechController.y();
    holdForSlowDrive = rightJoystick.button(2);
    holdForVFastDrive = leftJoystick.button(1);
    holdForVVFastDrive = rightJoystick.button(1);
    caliGyroTrig = rightJoystick.button(11);
    cubeScoreTrig = mechController.x();
    coneScoreTrig = mechController.b();
    cubeIntakeTrigger = mechController.leftBumper();
    coneIntakeTrigger = mechController.rightBumper();
    humanPlayerConeTrig = mechController.povUp();
    humanSecureTrig = mechController.povDown();
    humanPlayerIntakeTrig = mechController.povRight();
    manualWristBackwardsTrig = mechController.button(7);
    manualWristForwardTrig = mechController.button(8);
    repelClawConeTrigger = mechController.a(); //REDO
    repelClawCubeTrigger = mechController.povLeft(); //REDO
    mControlModeTrig = mechController.rightStick();
    
    
  }

  public SequentialCommandGroup getAutonomousCommand() {
    DriveDistance goOver = new DriveDistance(drive, 81, 0.3);
    DriveDistance backThatThangUp = new DriveDistance(drive, -70, -0.41);
    DriveDistance fallOff = new DriveDistance(drive, 40, 0.25);
    
    return new SequentialCommandGroup(scoreConeAuto(), goOver, fallOff, backThatThangUp.until(drive::botOnRamp) );
  }

  private SequentialCommandGroup scoreConeAuto(){
    return new SequentialCommandGroup(
      new InstantCommand(arm::brake),
      new InstantCommand(drive::brake),
      new InstantCommand(wrist::brake),
      new ParallelCommandGroup(
        new ArmToAngle(arm, -190).until(arm::atLevel), 
        new WristToAngle(wrist, 106).until(wrist::controllerAtSetPoint),
        new RunCommand(()->claw.setPowerAuto(-0.05), claw).withTimeout(1)).withTimeout(1.5),
      new SequentialCommandGroup(
        new RunCommand(()->claw.setPowerAuto(0.17), claw).withTimeout(0.9), 
        new RunCommand(()->claw.setPowerAuto(0), claw)).withTimeout(0.9),
       new ParallelCommandGroup(
        new ArmToAngle(arm, -30).until(arm::atLevel),
        new WristToAngle(wrist, 30).until(wrist::controllerAtSetPoint),
        new RunCommand(()->claw.setPowerAuto(0.35), claw).withTimeout(0.8)).withTimeout(4),
      new InstantCommand(()->claw.setPowerAuto(0))
    );
  }

  public Command getArcade(){
    return arcadeDrive;
  }
  public Command getManualArmCommand(){
    return powerArm;
  }
 
  public Command getStop(){
    return new InstantCommand(()->drive.set(0), drive);
  }
  public ParallelCommandGroup secureCone(){
    return new ParallelCommandGroup(new ArmToAngle(arm, -5), new WristToAngle(wrist, 30));
  }
}
