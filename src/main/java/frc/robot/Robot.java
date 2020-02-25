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
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.util.Color;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.*;

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

  //drivetrain stuff
  WPI_TalonSRX frontLeft;
	WPI_VictorSPX backRight;
	WPI_TalonSRX frontRight;
  WPI_VictorSPX backLeft;

  SpeedControllerGroup leftMotors;
  SpeedControllerGroup rightMotors;

  DifferentialDrive drive;
	
  Joystick driveStick;

  //shooter motors
  WPI_VictorSPX shooterTop;
  WPI_VictorSPX shooterBot;

  //map of port numbers
  Map map = new Map();
  
  //for determining if drivetrain going forward/backward
  public int LEFT_SIGN_MULTIPLIER;
  public int RIGHT_SIGN_MULTIPLIER;  

  //vision booleans
  boolean isClose;
  boolean isAlligningRight;
  boolean isAlligningLeft;
  
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

  //for vision alligning
  double previousArea = 0;
  double previousSkew = 0;

  //different stages of auton
  enum AutoStage{
    begin, shootOne, turnOne, driveOne, turnTwo, driveTwo, end
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

  DigitalInput limitIntake;
  AnalogInput beamBreakSensor1;
  AnalogInput beamBreakSensor2;
  AnalogInput beamBreakSensor3;
  AnalogInput beamBreakSensor4;

  @Override
  public void robotInit() {

    m_autoChooser.setDefaultOption("Center Auto", kCenterAuto);
    m_autoChooser.addOption("Left Auto", kLeftAuto);
    m_autoChooser.addOption("Right Auto", kRightAuto);
    SmartDashboard.putData("Auto Choices", m_autoChooser);    

    backRight = new WPI_VictorSPX(map.BACK_RIGHT_MOTOR_PORT);
    frontLeft = new WPI_TalonSRX(map.FRONT_LEFT_MOTOR_PORT);
		frontRight = new WPI_TalonSRX(map.FRONT_RIGHT_MOTOR_PORT);
    backLeft = new WPI_VictorSPX(map.BACK_LEFT_MOTOR_PORT);

		leftMotors = new SpeedControllerGroup (frontLeft, backLeft);
    rightMotors = new SpeedControllerGroup (frontRight, backRight);

    shooterTop = new WPI_VictorSPX(map.TOP_SHOOTER_PORT);
    shooterBot = new WPI_VictorSPX(map.BOT_SHOOTER_PORT);

    beamBreakSensor1 = new AnalogInput(map.BBS_1_PORT);
    beamBreakSensor2 = new AnalogInput(map.BBS_2_PORT);
    beamBreakSensor3 = new AnalogInput(map.BBS_3_PORT);
    beamBreakSensor4 = new AnalogInput(map.BBS_4_PORT);

    limitIntake = new DigitalInput(map.LIMIT_INTAKE_PORT);

    imu = new AHRS(SPI.Port.kMXP);

    colorMatcher.addColorMatch(kBlueTarget);
    colorMatcher.addColorMatch(kGreenTarget);
    colorMatcher.addColorMatch(kRedTarget);
    colorMatcher.addColorMatch(kYellowTarget);
		
		drive = new DifferentialDrive (leftMotors, rightMotors);
		drive.setSafetyEnabled(false);    

    driveStick = new Joystick(map.DRIVESTICK_PORT);

    stage = AutoStage.begin;

    isClose = false;
    gyroIsReset = false; 
    isAlligningLeft = false;
    isAlligningRight = false;

    conveyorIsRunning = false;
    preppingShoot = false;

    intakeIsRunning = false;

    checkingForPosCtrl = false;
    checkingForRotCtrl = false;
  }
  @Override
  public void robotPeriodic() {
    //for getting color sent during competitions
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

    if(driveStick.getRawButton(map.DRIVE_STICK_INTAKE)){
      if(!intakeIsRunning){
        intake(true);
        intakeIsRunning = true;
      }
      else{
        intake(false);
        intakeIsRunning = false;
      }
    }
  }
  @Override
  public void autonomousInit() {
    m_autoSelected = m_autoChooser.getSelected();
    System.out.println("Auto selected: " + m_autoSelected);
    resetGyro();   
    intake(true);
   
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
        break;
      case kLeftAuto:
        break;
      default:
        // Put default auto code here
        break;
    }
  }

  @Override
  public void teleopPeriodic() {

    //visionTracking();

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
    
    if(driveStick.getRawAxis(4)>.15){
      shooterTop.set(Math.pow(driveStick.getRawAxis(4),4));
      shooterBot.set(-Math.pow(driveStick.getRawAxis(4),4));
    }
    else{
      shooterTop.set(0);
      shooterBot.set(0);
    }
    
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
    if(driveStick.getRawAxis(1)>.15 || driveStick.getRawAxis(5)>.15 || driveStick.getRawAxis(1)<-.15 || driveStick.getRawAxis(5)<-.15 ){
      drive.tankDrive(Math.pow((driveStick.getRawAxis(1)),4) * LEFT_SIGN_MULTIPLIER, Math.pow((driveStick.getRawAxis(5)),4) * RIGHT_SIGN_MULTIPLIER);
    }
    else if(!isAlligningRight && !isAlligningLeft){
      drive.tankDrive(0,0);
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
      //RUN COLOR WHEEL MOTORS
      if(detectedColorString.equals(colorChosenForRotCtrl)){
        countOfSameColor++;
      }
      if(countOfSameColor>=7){
        //STOP COLOR WHEEL MOTORS
        checkingForRotCtrl = false;
        countOfSameColor = 0;
      }
    }
    if(checkingForPosCtrl){
      //RUN COLOR WHEEL MOTORS * dirForPosCtrl
      if(detectedColorString.equals(posCtrlColor)){
        //STOP COLOR WHEEL MOTORS
        checkingForRotCtrl = false;
      }
    }
  }

  @Override
  public void testPeriodic() {
  }

  public void resetGyro(){
    imu.resetDisplacement();
    imu.reset();
  }
  public void visionTracking(){
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    NetworkTableEntry ts = table.getEntry("ts");

    //read values periodically
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);
    double skew = ts.getDouble(0.0);

    //post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
    SmartDashboard.putNumber("LimelightSkew", skew);

    SmartDashboard.putNumber("Angle", imu.getAngle());

    //being alligning once button is clicking based on orientation of robot
    if(driveStick.getRawButton(2)){
      if(imu.getAngle()<0){
        isAlligningRight = true;
      }
      else{
        isAlligningLeft = true;
      }
    }
    if(isAlligningRight){
      drive.tankDrive(.4,-.4);
      if(Math.abs(previousSkew-skew)>80){
        System.out.println("Stoppedqwertyuikolkjhgfxghjhbgfghjhgfcdfghjhgfghjhgvffghj");
        drive.tankDrive(0,0);
        isAlligningRight = false;
        previousSkew = 0;
      }
    }
    else if(isAlligningLeft){
      drive.tankDrive(-.4,.4);
      if(Math.abs(previousSkew-skew)>80){
        System.out.println("Stoppedqwertyuikolkjhgfxghjhbgfghjhgfcdfghjhgfghjhgvffghj");
        drive.tankDrive(0,0);
        isAlligningLeft = false;
        previousSkew = 0;
      }
    }
    previousArea = area;
    previousSkew = skew;
  }
public void centerAuto(){
  switch(stage){
    case begin:
      stage = AutoStage.shootOne;
      break;
    case shootOne:
      shooter(true);
      countsForShooter++;
      if(countsForShooter>=150){
        shooter(false);
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
      shooter(false);
      preppingShoot = false;
      countsForShooter = 0;
    }
    if(preppingShoot){
      countsForShooter++;
    }
    if(checkBeam(beamBreakSensor4)){
      //click shoot button
      if(driveStick.getRawButton(map.DRIVE_STICK_SHOOTER)){
        shooter(true);
        //move conveyors
        conveyorIsRunning = true;
        preppingShoot = true;
      }
      //turns on shooter if ball at top spot during auton
      else if(duringAuton&&!preppingShoot){
        shooter(true);
        //move conveyors
        conveyorIsRunning = true;
        preppingShoot = true;
      }
    }
    //runs motors if ball in intake cavity and no ball in top spot
    if(limitIntake.get()&&!checkBeam(beamBreakSensor4)){
      //move conveyors
      conveyorIsRunning = true;
    }
    if(conveyorIsRunning){
      //delay for when balls initially move
      countsForDelay++;
      if(countsForDelay>=50){
        //stops conveyor when any sensor is triggered
        if(checkBeam(beamBreakSensor4)||checkBeam(beamBreakSensor3)||checkBeam(beamBreakSensor2)||checkBeam(beamBreakSensor1)){
          //stop conveyor
          conveyorIsRunning = false;
          countsForDelay = 0;
        }
      }
    }
  }
  public void shooter(boolean turnOn){
    if(turnOn){

    }
    else{

    }
  }
  public void intake(boolean turnOn){
    if(turnOn){

    }
    else{
      
    }
  }
  public boolean checkBeam(AnalogInput sensor){
    if(sensor.getVoltage()>0.2){
      return true;
    }
    return false;
  }
}

