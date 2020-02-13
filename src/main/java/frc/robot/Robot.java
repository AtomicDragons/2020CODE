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
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private static final String kRightAuto = "Right Start";
  private static final String kCenterAuto = "Center Start";
  private static final String kLeftAuto = "Left Start";

  private final SendableChooser<String> m_autoChooser = new SendableChooser<>();

  AHRS imu;

  WPI_TalonSRX frontLeft;
	WPI_VictorSPX backRight;
	WPI_TalonSRX frontRight;
  WPI_VictorSPX backLeft;

  WPI_VictorSPX outtakeLeft;
  WPI_VictorSPX outtakeRight;

  SpeedControllerGroup leftMotors;
  SpeedControllerGroup rightMotors;
  SpeedControllerGroup wheelMotors;

  Map map = new Map();
  
	
  DifferentialDrive drive;
	
  Joystick driveStick;

  public int LEFT_SIGN_MULTIPLIER;
  public int RIGHT_SIGN_MULTIPLIER;  

  
  boolean isClose;
  boolean isAlligningRight;
  boolean isAlligningLeft;

  boolean gyroIsReset;

  double previousArea = 0;
  double previousSkew = 0;

 
  enum AutoMode{
    kLeftAuto, kCenterAuto, kRightAuto
  }
  enum AutoStage{
    turnOne, driveOne
  }

  AutoMode mode;
  AutoStage stage;

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {

    isClose = false;
    gyroIsReset = false; 
    isAlligningLeft = false;
    isAlligningRight = false;

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

    outtakeLeft = new WPI_VictorSPX(map.LEFT_OUTTAKE_PORT);
    outtakeRight = new WPI_VictorSPX(map.RIGHT_OUTTAKE_PORT);

    imu = new AHRS(SPI.Port.kMXP);

		
		drive = new DifferentialDrive (leftMotors, rightMotors);
		drive.setSafetyEnabled(false);    

    driveStick = new Joystick(map.DRIVESTICK_PORT);

 
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
   
    switch (m_autoSelected) {
      case kCenterAuto:
        // Put custom auto code here
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

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {

    visionTracking();

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
      outtakeRight.set(Math.pow(driveStick.getRawAxis(4),4));
      outtakeLeft.set(-Math.pow(driveStick.getRawAxis(4),4));
    }
    else{
      outtakeRight.set(0);
      outtakeLeft.set(0);
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

    
    if(driveStick.getRawAxis(1)>.15 || driveStick.getRawAxis(5)>.15 || driveStick.getRawAxis(1)<-.15 || driveStick.getRawAxis(5)<-.15 ){
      drive.tankDrive(Math.pow((driveStick.getRawAxis(1)),4) * LEFT_SIGN_MULTIPLIER, Math.pow((driveStick.getRawAxis(5)),4) * RIGHT_SIGN_MULTIPLIER);
    }
    else if(!isAlligningRight && !isAlligningLeft){
      drive.tankDrive(0,0);
    }
    
    
  }

  /**
   * This function is called periodically during test mode.
   */
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

    System.out.println(skew);
    //System.out.println(Math.abs(previousSkew-skew));
    //System.out.println(area);
    if(driveStick.getRawButton(2)){
      System.out.println("button hello");
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
}
