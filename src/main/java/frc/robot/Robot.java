/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Compressor;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.util.Color;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.*;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.wpilibj.AnalogInput;     

public class Robot extends TimedRobot {

  //For selecting different auton strategies
  private static final String kRightAuto = "Right Start";
  private static final String kCenterAuto = "Center Start";
  private static final String kLeftAuto = "Left Start";

  private final SendableChooser<String> m_autoChooser = new SendableChooser<>();
  private String m_autoSelected;

  //gyro
  AHRS imu;
  
  //belly
  CANSparkMax belly_1;
  CANSparkMax belly_2;
  CANSparkMax shooterTop;
  CANSparkMax shooterBottom;
  CANSparkMax intake;
  CANSparkMax colorWheel;
  CANSparkMax climberArm;

  //drivetrain stuff
  WPI_TalonFX rightFront;
  WPI_TalonFX rightCenter;
  WPI_TalonFX rightBack;
  WPI_TalonFX leftFront;
  WPI_TalonFX leftCenter;
  WPI_TalonFX leftBack;

  Faults faultsR = new Faults();
  Faults faultsL = new Faults();

  DifferentialDrive drive;
	
  Joystick driveStick;

  //map of port numbers
  Map map = new Map();
  
  //for determining if drivetrain going forward/backward
  public int LEFT_SIGN_MULTIPLIER;
  public int RIGHT_SIGN_MULTIPLIER;  

  //vision
  boolean isClose;
  boolean targetIsInframe;
  boolean isAlligning;
  
  //for vision alligning
  double previousArea = 0;
  double previousSkew = 0;

  //intake
  boolean intakeIsRunning;

  int countsToTurnOnIntakeDuringAuton = 0;

  //color wheel
  boolean checkingForRotCtrl;
  boolean checkingForPosCtrl;

  int countOfSameColor = 0;
  String colorChosenForRotCtrl = "";
  int dirForPosCtrl = 1;

  String[] colors = {"Red", "Green", "Blue", "Yellow"};

  //belly
  boolean conveyorIsRunning;
  boolean preppingShoot;

  int countsForShooter = 0;
  int countsForDelay = 0;

  //boolean for testing 
  boolean gyroIsReset;

  //different stages of auton
  enum AutoStage{
    begin, allign, shootOne, turnOne, driveOne, turnTwo, driveTwo, end
  }

  AutoStage stage;

  //count for how long robot is in acceptble threshhold for pid
  int countsInThreshold = 0;

  //color sensor stuff
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  //IMPORTANT:MAKE SURE LED ON COLOR SENSOR IS OFF
  private final ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);
  private final ColorMatch colorMatcher = new ColorMatch();

  private final Color kBlueTarget = ColorMatch.makeColor(0.202,0.483,0.314);
  private final Color kGreenTarget = ColorMatch.makeColor(0.252,0.563,0.184);
  private final Color kRedTarget = ColorMatch.makeColor(0.638,0.294,0.067);
  private final Color kYellowTarget = ColorMatch.makeColor(0.428,0.492,0.079);

  //color given by game
  String posCtrlColor = "";

  //current color picked up by sensor
  String detectedColorString;

  //sensors
  DigitalInput limitIntake;
  AnalogInput beamBreakSensor1;
  AnalogInput beamBreakSensor2;
  AnalogInput beamBreakSensor3;
  AnalogInput beamBreakSensor4;

  //vision data
  NetworkTable table;
  NetworkTableEntry tx;
  NetworkTableEntry ty;
  NetworkTableEntry ta;
  NetworkTableEntry ts;

  double vision_x;
  double vision_y;
  double vision_area;
  double vision_skew;

  //solenoids
  DoubleSolenoid colorWheelSully;
  DoubleSolenoid releaseClimberSully;
  DoubleSolenoid highGearSully1;
  DoubleSolenoid highGearSully2;
  DoubleSolenoid climbUpSully1;
  DoubleSolenoid climbUpSully2;

  Compressor compressor;

  boolean isHighGear;
  boolean colorWheelArmIsRaised;

  //climber
  int countsForPiston = 0;
  int countsForClimberMotor = 0;
  
  @Override
  public void robotInit() {

    m_autoChooser.setDefaultOption("Center Auto", kCenterAuto);
    m_autoChooser.addOption("Left Auto", kLeftAuto);
    m_autoChooser.addOption("Right Auto", kRightAuto);
    SmartDashboard.putData("Auto Choices", m_autoChooser);    

    /*
    rightFront = new WPI_TalonFX(map.FRONT_RIGHT_MOTOR_PORT);
    rightCenter = new WPI_TalonFX(map.CENTER_RIGHT_MOTOR_PORT);
    rightBack = new WPI_TalonFX(map.BACK_RIGHT_MOTOR_PORT);
    leftFront = new WPI_TalonFX(map.FRONT_LEFT_MOTOR_PORT);
    leftCenter = new WPI_TalonFX(map.CENTER_LEFT_MOTOR_PORT);
    leftBack = new WPI_TalonFX(map.BACK_LEFT_MOTOR_PORT);

    rightFront.configFactoryDefault();
    rightCenter.configFactoryDefault();
    rightBack.configFactoryDefault();
    leftFront.configFactoryDefault();
    leftCenter.configFactoryDefault();
    leftBack.configFactoryDefault();

    rightCenter.follow(rightFront);
    rightBack.follow(rightFront);
    leftCenter.follow(leftFront);
    leftBack.follow(leftFront);

    rightFront.setInverted(TalonFXInvertType.Clockwise);
    leftFront.setInverted(TalonFXInvertType.Clockwise);

    rightCenter.setInverted(TalonFXInvertType.FollowMaster);
    rightBack.setInverted(TalonFXInvertType.FollowMaster);
    leftCenter.setInverted(TalonFXInvertType.FollowMaster);
    leftBack.setInverted(TalonFXInvertType.FollowMaster);
    */

    belly_1 = new CANSparkMax(map.BELLY_1_MOTOR_PORT, MotorType.kBrushless);
    belly_2 = new CANSparkMax(map.BELLY_2_MOTOR_PORT, MotorType.kBrushless);
    shooterTop = new CANSparkMax(map.TOP_SHOOTER_MOTOR_PORT, MotorType.kBrushless);
    shooterBottom = new CANSparkMax(map.BOT_SHOOTER_MOTOR_PORT, MotorType.kBrushless);
    intake = new CANSparkMax(map.INTAKE_MOTOR_PORT, MotorType.kBrushless);

    /*
    colorWheel = new CANSparkMax(map.COLOR_WHEEL_MOTOR_PORT, MotorType.kBrushless);
    climberArm = new CANSparkMax(map.CLIMBER_ARM_MOTOR_PORT, MotorType.kBrushless);
    */
  
    
    belly_1.restoreFactoryDefaults();
    belly_2.restoreFactoryDefaults();
    shooterTop.restoreFactoryDefaults();
    shooterBottom.restoreFactoryDefaults();
    intake.restoreFactoryDefaults();
    /*
    colorWheel.restoreFactoryDefaults();
    climberArm.restoreFactoryDefaults();
    */
  
    beamBreakSensor1 = new AnalogInput(map.BBS_1_PORT);
    beamBreakSensor2 = new AnalogInput(map.BBS_2_PORT);
    beamBreakSensor3 = new AnalogInput(map.BBS_3_PORT);
    beamBreakSensor4 = new AnalogInput(map.BBS_4_PORT);

    //compressor = new Compressor(map.COMPRESSOR_PORT);

    //CHANGE THE PARAMETERS!!!!!!!!!<==============================================
    /*
    colorWheelSully = new DoubleSolenoid(0,map.COLOR_WHEEL_SULLY_PORT, map.COLOR_WHEEL_SULLY_PORT + 1);
    releaseClimberSully = new DoubleSolenoid(10,map.RELEASE_CLIMBER_SULLY_PORT,map.RELEASE_CLIMBER_SULLY_PORT+1);
    highGearSully1 = new DoubleSolenoid(10,map.HIGH_GEAR_SULLY_1_PORT,map.HIGH_GEAR_SULLY_1_PORT+1);
    highGearSully2 = new DoubleSolenoid(10,map.HIGH_GEAR_SULLY_2_PORT,map.HIGH_GEAR_SULLY_2_PORT+1);
    climbUpSully1 = new DoubleSolenoid(10,map.CLIMB_UP_SULLY_1_PORT,map.CLIMB_UP_SULLY_1_PORT+1);
    climbUpSully2 = new DoubleSolenoid(10,map.CLIMB_UP_SULLY_2_PORT,map.CLIMB_UP_SULLY_2_PORT+1);
    */
    limitIntake = new DigitalInput(map.LIMIT_INTAKE_PORT);

    imu = new AHRS(SPI.Port.kMXP);

    colorMatcher.addColorMatch(kBlueTarget);
    colorMatcher.addColorMatch(kGreenTarget);
    colorMatcher.addColorMatch(kRedTarget);
    colorMatcher.addColorMatch(kYellowTarget);
		
		//drive = new DifferentialDrive (leftFront, rightFront);
    //drive.setSafetyEnabled(false);    
    //drive.setRightSideInverted(false);

    driveStick = new Joystick(map.DRIVESTICK_PORT);

    stage = AutoStage.begin;

    isClose = false;
    gyroIsReset = false; 
    isAlligning = false;

    conveyorIsRunning = false;
    preppingShoot = false;

    intakeIsRunning = false;

    checkingForPosCtrl = false;
    checkingForRotCtrl = false;

    targetIsInframe = false;

    isHighGear = false;
    colorWheelArmIsRaised = false;
 
    table = NetworkTableInstance.getDefault().getTable("limelight");
  }
  @Override
  public void robotPeriodic() {

    //visionTracking();
    //for getting color sent during competitions
    /*
    String gameData;
    gameData = DriverStation.getInstance().getGameSpecificMessage();
    if(gameData.length()>0){
      switch(gameData.charAt(0)){
        case 'R':
          posCtrlColor = "Red";
          break;
        case 'G':
          posCtrlColor = "Green";
          break;
        case 'B':
          posCtrlColor = "Blue";
          break;
        case 'Y':
          posCtrlColor = "Yellow";
          break;
        default:
          break;
      }
    }
    Color detectedColor = colorSensor.getColor();

    ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);

    if(match.color == kBlueTarget){
      detectedColorString = "Blue";
    }
    else if(match.color == kRedTarget){
      detectedColorString = "Red";
    }
    else if(match.color == kGreenTarget){
      detectedColorString = "Green";
    }
    else if(match.color == kYellowTarget){
      detectedColorString = "Yellow";
    }
    else{
      detectedColorString = "Unknown";
    }
    
    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putString("Detected Color", detectedColorString);
    SmartDashboard.putNumber("BEAM", beamBreakSensor1.getVoltage());
    */
   
    /*
    rightFront.getFaults(faultsR);
    leftFront.getFaults(faultsL);

    if(faultsR.SensorOutOfPhase){
      System.out.println("R SENSOR IS OUT OF PHASE!!!");
    }
    if(faultsL.SensorOutOfPhase){
      System.out.println("L SENSOR IS OUT OF PHASE!!!");
    }
    */
  }
  @Override
  public void autonomousInit() {
    m_autoSelected = m_autoChooser.getSelected();
    System.out.println("Auto selected: " + m_autoSelected);
    resetGyro();   
  }
  @Override
  public void autonomousPeriodic() {
   
    countsToTurnOnIntakeDuringAuton++;

    if(countsToTurnOnIntakeDuringAuton>=500){
      intake(false);
    }
    switch (m_autoSelected) {
      case kCenterAuto:
        centerAuto();
        break;
      case kRightAuto:
        rightAuto();
        break;
      case kLeftAuto:
        leftAuto();
        break;
      default:
        break;
    }
   // updateConveyor(true);
  }
  @Override
  public void teleopPeriodic() {

    if(driveStick.getRawButton(14)){   
      setConveyor(.5);
    }

    if(driveStick.getRawAxis(1)<0){
      LEFT_SIGN_MULTIPLIER = 1;
    }
    else{
      LEFT_SIGN_MULTIPLIER = -1;
    }

    if(driveStick.getRawAxis(5)<0){
      RIGHT_SIGN_MULTIPLIER = 1;
    }
    else{
      RIGHT_SIGN_MULTIPLIER = -1;
    }
    /*
    if(driveStick.getRawAxis(4)>.15){
      shoot(driveStick.getRawAxis(4));
    }
    else{
      shoot(0);
    }
    */
    if(!gyroIsReset){
      resetGyro();
      gyroIsReset = true;
    }
    /*
    if(!isClose && y<-6){
      drive.tankDrive(.3, .3);
    }
    else{
      isClose = true;
      drive.tankDrive(0,0);
    }
    */

    //for operating tank drive
    /*
    if(driveStick.getRawAxis(1)>.15 || driveStick.getRawAxis(5)>.15 || driveStick.getRawAxis(1)<-.15 || driveStick.getRawAxis(5)<-.15 ){
      drive.tankDrive(Math.pow((driveStick.getRawAxis(1)),4) * LEFT_SIGN_MULTIPLIER, Math.pow((driveStick.getRawAxis(5)),4) * RIGHT_SIGN_MULTIPLIER);
    }
    else if(!isAlligning){
      drive.tankDrive(0,0);
    }
    */

    if(driveStick.getPOV()==map.DRIVE_STICK_TOGGLE_GEAR){
      if(!isHighGear){
        highGearSully1.set(DoubleSolenoid.Value.kForward);
        highGearSully2.set(DoubleSolenoid.Value.kForward);
      }
      else{
        highGearSully1.set(DoubleSolenoid.Value.kReverse);
        highGearSully2.set(DoubleSolenoid.Value.kReverse);
      }
    }
    if(driveStick.getPOV()==map.DRIVE_STICK_RAISE_COLOR_WHEEL_ARM){
      if(!colorWheelArmIsRaised){
        colorWheelSully.set(DoubleSolenoid.Value.kForward);
        colorWheelArmIsRaised = true;
      }
      else{
        colorWheelSully.set(DoubleSolenoid.Value.kReverse);
        colorWheelArmIsRaised = false;
      }
    }
    if(driveStick.getPOV()==map.RELEASE_CLIMBER_SULLY_PORT){
      deployClimber();
    }

    //rot control
    if(driveStick.getRawButton(map.DRIVE_STICK_ROT_CTRL)&&detectedColorString!="Unknown"){
        checkingForRotCtrl = true;
        colorChosenForRotCtrl = detectedColorString;
    }
    //pos control
    if(driveStick.getRawButton(map.DRIVE_STICK_POS_CTRL)&&detectedColorString!="Unknown"){
      checkingForPosCtrl = true;
      int currentColorPos = 0;
      int targetColorPos = 0;
      //figures out the optimal direction to turn the wheel to the target color
      for(int i = 0;i<colors.length;i++){
        if(colors[i].equals(detectedColorString)){
          currentColorPos = i;
        }
        else if(colors[i].equals(posCtrlColor)){
          targetColorPos = i;
        }
      }
      int distanceRight = targetColorPos - currentColorPos;
      int distanceLeft = currentColorPos - targetColorPos;
      if(distanceRight == -3) distanceRight = 1;
      else if(distanceRight == -2) distanceRight = 2;
      else if(distanceRight == -1) distanceRight = 3;
      if(distanceLeft == -3) distanceLeft = 1;
      else if(distanceLeft == -2) distanceLeft = 2;
      else if(distanceLeft == -1) distanceLeft = 3;

      if(distanceRight<=distanceLeft){
        dirForPosCtrl = -1;
      }
      else{
        dirForPosCtrl = 1;
      }
    }
    if(checkingForRotCtrl){
      colorWheel.set(.5);
      if(detectedColorString.equals(colorChosenForRotCtrl)){
        countOfSameColor++;
      }
      if(countOfSameColor>=7){
        colorWheel.set(0);
        checkingForRotCtrl = false;
        countOfSameColor = 0;
      }
    }
    if(checkingForPosCtrl){
      colorWheel.set(.3 * dirForPosCtrl);
      if(detectedColorString.equals(posCtrlColor)){
        colorWheel.set(-.1 * dirForPosCtrl);
        checkingForRotCtrl = false;
      }
    }
    if(driveStick.getRawButton(map.DRIVE_STICK_INTAKE)){
      if(!intakeIsRunning){
        intake(true);
      }
      else{
        intake(false);
      }
    }
    //toggle to turn on/off limelight LED
    if(driveStick.getRawButton(map.DRIVE_STICK_TOGGLE_LED)){
      //turn off
      if(table.getEntry("ledMode").getDouble(0)==3){
        table.getEntry("ledMode").setNumber(1);
      }
      //turn on
      else{
        table.getEntry("ledMode").setNumber(3);
      }
    }
    //updateConveyor(false);
  }

  @Override
  public void testPeriodic() {
    
  }

  public void resetGyro(){
    imu.resetDisplacement();
    imu.reset();
  }
  public void visionTracking(){

    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");
    ts = table.getEntry("ts");

    //read values periodically
    vision_x = tx.getDouble(0.0);
    vision_y = ty.getDouble(0.0);
    vision_area = ta.getDouble(0.0);
    vision_skew = ts.getDouble(0.0);

    //post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", vision_x);
    SmartDashboard.putNumber("LimelightY", vision_y);
    SmartDashboard.putNumber("LimelightArea", vision_area);
    SmartDashboard.putNumber("LimelightSkew", vision_skew);

    SmartDashboard.putNumber("Angle", imu.getAngle());

    //alligning once button is clicking based on orientation of robot
    if(driveStick.getRawButton(map.DRIVE_STICK_ALLIGN_LEFT)){   
      isAlligning = true;
      if(!allign("Left")){
        isAlligning = false;
      }
    }
    else if(driveStick.getRawButton(map.DRIVE_STICK_ALLIGN_RIGHT)){ 
      isAlligning = true;  
      if(!allign("Right")){
        isAlligning = false;
      }
    }
  }
  public void rightAuto(){
    switch(stage){
      case begin:
        stage = AutoStage.allign;
        break;
      case allign:
        if(!allign("Left")){
        }
        else{
          stage = AutoStage.shootOne;
        }
        break;
      case shootOne:
        shoot(1);
        countsForShooter++;
        if(countsForShooter>=150){
          shoot(0);
          countsForShooter = 0;
          stage = AutoStage.driveOne;
        }
        break;
      case driveOne:
        if(!driveToDistance(-90)){
        }
        else{
          stage = AutoStage.end;
        }
        break;
      case end:
        break;
      default:
        break;
    }
  }
  public void centerAuto(){
    switch(stage){
      case begin:
        stage = AutoStage.shootOne;
        break;
      case shootOne:
        shoot(1);
        countsForShooter++;
        if(countsForShooter>=150){
          shoot(0);
          countsForShooter = 0;
          stage = AutoStage.turnOne;
        }
        break;
      case turnOne:
        if(!turnToAngle(-90)){
        }
        else{
          stage = AutoStage.driveOne;
        }
        break;
      case driveOne:
        if(!driveToDistance(169.95)){
        }
        else{
          stage = AutoStage.turnTwo;
        }
        break;
      case turnTwo:
        if(!turnToAngle(90)){
        }
        else{
          stage = AutoStage.driveTwo;
        }
        break;
      case driveTwo:
        if(!driveToDistance(495)){
          intake(true);
        }
        else{
          intake(false);
          stage = AutoStage.end;
        }
        break;
      case end:
        break;
      default:
        break;
    }
  }
  public void leftAuto(){
    switch(stage){
      case begin:
        stage = AutoStage.allign;
        break;
      case allign:
        if(!allign("Right")){
        }
        else{
          stage = AutoStage.shootOne;
        }
        break;
      case shootOne:
        shoot(1);
        countsForShooter++;
        if(countsForShooter>=150){
          shoot(0);
          countsForShooter = 0;
          stage = AutoStage.driveOne;
        }
        break;
      case driveOne:
        if(!driveToDistance(-90)){
        }
        else{
          stage = AutoStage.end;
        }
        break;
      case end:
        break;
      default:
        break;
    }
  } 
  public boolean driveToDistance(double targetDistance){
    return false;
  }
  public boolean turnToAngle(double targetAngle){
    double currentAngle = imu.getAngle();
    double error = targetAngle - currentAngle;
    double fullSpeedThreshold = 30;
    double targetThreshold = 5;
    int dir;
    double kp = .02;

    //makes direction positive if target angle is negative
    if(targetAngle < 0) {
      dir = 1;
    } else {
      dir = -1;
    }

    //changes direction if robot overshoots target
    if((targetAngle>0&&error<0)||(targetAngle<0&&error>0)){
      dir = -dir;
    }
    //"full" speed if angle above full speed threshold
    if(Math.abs(error) > fullSpeedThreshold){
      drive.tankDrive(-.5 * dir, .5 * dir);
      return false;
    }
    //pid speed if angle above acceptable targetThreshold
    else if(Math.abs(error) > targetThreshold){
      drive.tankDrive(-Math.abs(error) * kp * dir, Math.abs(error) * kp * dir);
      return false;
    }
    //stops robot
    else{     
      drive.tankDrive(0, 0);
      countsInThreshold++;
      //if robot in threshold for 5 iterations, method terminates
      if(countsInThreshold==5){
        return true;
      }
      return false;
    }
  }
  public void updateConveyor(boolean duringAuton){
    //after 3 seconds, ball has been shot
    if(countsForShooter >= 150){//test actual time it takes to shoot ball from falling into ramp
      shoot(0);
      preppingShoot = false;
      countsForShooter = 0;
    }
    if(preppingShoot){
      countsForShooter++;
    }
    if(checkBeam(beamBreakSensor4)){
      //click shoot button
      if(driveStick.getRawButton(map.DRIVE_STICK_SHOOTER)&&!preppingShoot){
        shoot(1);
        setConveyor(1);
        preppingShoot = true;
      }
      //turns on shooter if ball at top spot during auton
      else if(duringAuton&&!preppingShoot){
        shoot(1);
        setConveyor(1);
        preppingShoot = true;
      }
    }
    //runs motors if ball in intake cavity and no ball in top spot
    if(driveStick.getPOV() == 90 && !checkBeam(beamBreakSensor4)){//CHANGE BACK TO limitIntake.get()
      setConveyor(1);
    }
    if(conveyorIsRunning){
      //delay for when balls initially move
      countsForDelay++;
      if(countsForDelay>=50){
        //stops conveyor when any sensor is triggered
        if(checkBeam(beamBreakSensor4)||checkBeam(beamBreakSensor3)||checkBeam(beamBreakSensor2)||checkBeam(beamBreakSensor1)){
          setConveyor(0);
          countsForDelay = 0;
        }
      }
    }
  }

  public void setConveyor(double power){
    if(power>0){
      conveyorIsRunning = true;
    }
    else{
      conveyorIsRunning = false;
    }
    belly_1.set(-power);
    belly_2.set(power);
  }

  public void shoot(double power){
    shooterTop.set(power);
    shooterBottom.set(-power);
  }

  public void intake(boolean isOn){
    if(isOn){
      intake.set(1);
    }
    else{
      intake.set(0);
    }
  }

  public boolean checkBeam(AnalogInput sensor){
    if(sensor.getVoltage()>0.2){
      return true;
    }
    return false;
  }
  
  public boolean allign(String direction){
    table.getEntry("ledMode").setNumber(3);
    int dir;
    if(direction.equals("Left")){
      dir = -1;
    }
    else{
      dir = 1;
    }
    if(!targetIsInframe){
      drive.tankDrive(-dir * .4,dir * .4);
      if(Math.abs(previousSkew-vision_skew)>80){
        System.out.println("Stoppedqwertyuikolkjhgfxghjhbgfghjhgfcdfghjhgfghjhgvffghj");
        drive.tankDrive(0,0);
        previousSkew = 0;
        targetIsInframe = true;
      }
    }
    previousArea = vision_area;
    previousSkew = vision_skew;

    if(targetIsInframe){
      drive.tankDrive(-dir * .4,dir * .4);  
      if(Math.abs(vision_x)<5){
        System.out.println("Stoppedqwertyuikolkjhgfxghjhbgfghjhgfcdfghjhgfghjhgvffghj");
        drive.tankDrive(0,0);
        table.getEntry("ledMode").setNumber(1);
        return true;
      }
      return false;
    }
    return false;
  }
  public void deployClimber(){
    releaseClimberSully.set(DoubleSolenoid.Value.kForward);
    countsForPiston++;
    if(countsForPiston>20){
      climberArm.set(.5);
      countsForClimberMotor++;
    }
    if(countsForClimberMotor>30){
      climbUpSully1.set(DoubleSolenoid.Value.kForward);
      climbUpSully2.set(DoubleSolenoid.Value.kForward);
    } 
  }
}

