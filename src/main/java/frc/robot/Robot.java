// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  int state = 0;
  int debounceCount = 0;
  double onChargeStationDegree = 13.0;
  double debounceTime = 0.2;
  double robotSpeedSlow = 0.325;
  double robotSpeedFast = 0.65;
  double levelDegree = 6.0;
  boolean toggle = false;
  boolean up = false;
  boolean down = false;
  int autoCounter = 0;
  final double kp = 0.01;
  final double ki = 0.00;
  final double iLimit = 2;
  final double kd = 0.00;
  double dt = 0.0;
  double sensorposition = 0.0;
  double error = 0.0;
  double errorSum = 0.0;
  double errorRate = 0.0;
  double lasterror = 0.0;
  double lastTimestamp = 0.0;
  double lastpitch = 0.0;
  boolean mid = false;
  double slowdrive = 1.0;
  double slowturn = 1.0;
  double high = -219;
  double middle = -116;
  int angle = 5;
  boolean turn = false;
  boolean align = false;
  double turnsetpoint = 0.0;
  private static final String kDefaultAuto = "Cube + Engage";
  private static final String kCustomAuto = "Cube + Mobility";
  private static final String kCustomAuto1 = "Cone + Mobility";

  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private final Timer timer = new Timer();
  private final Timer intaketimer = new Timer();
  private final Timer elevatortimer = new Timer();



  final Joystick JOYSTICK = new Joystick(0);
  final Joystick JOYSTICK2 = new Joystick(1);

  
  final WPI_VictorSPX FRONT_LEFT = new WPI_VictorSPX(4);
  final WPI_VictorSPX FRONT_RIGHT = new WPI_VictorSPX(6);
  final WPI_VictorSPX BACK_LEFT = new WPI_VictorSPX(12);
  final WPI_VictorSPX BACK_RIGHT = new WPI_VictorSPX(5);
  final WPI_VictorSPX ROTATION = new WPI_VictorSPX(13);


  final MotorControllerGroup LEFT_DRIVE = new MotorControllerGroup(FRONT_LEFT, BACK_LEFT);
  final MotorControllerGroup RIGHT_DRIVE = new MotorControllerGroup(FRONT_RIGHT, BACK_RIGHT);
  final Encoder encoderA = new Encoder(1, 2);
  // final Encoder encoderB = new Encoder(2, 3);

  final DifferentialDrive ROBOT_DRIVE = new DifferentialDrive(LEFT_DRIVE, RIGHT_DRIVE);

  final CANSparkMax m_leftElevator = new CANSparkMax(1, MotorType.kBrushless);
  final CANSparkMax m_rightElevator = new CANSparkMax(2, MotorType.kBrushless);
  final CANSparkMax INTAKE = new CANSparkMax(3, MotorType.kBrushless);

  final RelativeEncoder m_elevatorEncoder = m_rightElevator.getEncoder();


  final MotorControllerGroup m_elevator = new MotorControllerGroup(m_leftElevator, m_rightElevator);
  final AHRS gyro = new AHRS(SPI.Port.kMXP);
  final DigitalInput toplimitswitch = new DigitalInput(0);

  public Robot() {
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    CameraServer.startAutomaticCapture();
    m_rightElevator.setInverted(true);
    RIGHT_DRIVE.setInverted(true);
    m_chooser.setDefaultOption("Cube + Engage", kDefaultAuto);
    m_chooser.addOption("Cube + Mobility", kCustomAuto);
    m_chooser.addOption("Cone + Mobility", kCustomAuto1);

    SmartDashboard.putData("Auto choices", m_chooser);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    FRONT_LEFT.setNeutralMode(NeutralMode.Brake);
    FRONT_RIGHT.setNeutralMode(NeutralMode.Brake);
    BACK_LEFT.setNeutralMode(NeutralMode.Brake);
    BACK_RIGHT.setNeutralMode(NeutralMode.Brake);
    state = 0;
    debounceCount = 0;
    autoCounter = 0;
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
    timer.reset();
    timer.start();
    encoderA.reset();
    // encoderB.reset();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
        case kCustomAuto1:
          if (autoCounter == 0) {
            if (timer.get() < 0.2) {
              ROTATION.set(-0.5);
            } else {
              ROTATION.set(0.0);
              timer.reset();
              autoCounter++;
            }
          }
          if (autoCounter == 1) {
            if (!toplimitswitch.get()) {
              m_elevator.set(-0.8);
            } else if (toplimitswitch.get()) {
              m_elevator.set(0.0);
              timer.reset();
              autoCounter++;
            } else {
              autoCounter = 8;
              System.out.println("error");
            }
          }
          if (autoCounter == 2) {
            if (timer.get() < 0.1) {
              m_elevator.set(0.3);
            } else {
              m_elevator.set(0.0);
              timer.reset();
              autoCounter++;
            }
          }
          if (autoCounter == 3) {
            if (timer.get() < 1.5) {
              ROBOT_DRIVE.arcadeDrive(-0.45, 0.0);
            } else {
              timer.reset();
              autoCounter++;
            }
          }
          if (autoCounter == 5) {
            if (timer.get() < 1) {
              INTAKE.set(0.5);
            } else {
              INTAKE.set(0.0);
              timer.reset();
              autoCounter++;
            }
          }
          if (autoCounter == 4) {
            if (timer.get() < 0.7) {
              ROTATION.set(-0.6);
            } else {
              ROTATION.set(0.0);
              timer.reset();
              autoCounter++;
            }
          }
          if (autoCounter == 6) {
            if (timer.get() < 0.5) {
              ROTATION.set(0.6);
            } else {
              timer.reset();
              autoCounter++;
            }
          }
          if (autoCounter == 7) {
            if (timer.get() < 1.4) {
              m_elevator.set(0.7);
            } else {
              m_elevator.set(0.0);
              timer.reset();
              autoCounter++;
            }
          }
          if (autoCounter == 8) {
            if (timer.get() < 3) {
              ROBOT_DRIVE.arcadeDrive(0.6, 0.0);
            } else if (encoderA.getDistance() < 2800) {
              ROBOT_DRIVE.arcadeDrive(0.55, 0.0);
            } else {
              ROBOT_DRIVE.arcadeDrive(0.0, 0.0);
              autoCounter++;
              encoderA.reset();
            }
          }
          if (autoCounter == 9) {
            if (encoderA.getDistance() < 430) {
              ROBOT_DRIVE.arcadeDrive(0.0, 0.4);
            } else {
              ROBOT_DRIVE.arcadeDrive(0.0, 0.0);
              autoCounter++;
            }
          }
          break;

        case kCustomAuto:
          if (autoCounter == 0) {
            if (timer.get() < 0.3) {
              ROTATION.set(-0.5);
            } else {
              ROTATION.set(0.0);
              timer.reset();
              autoCounter++;
            }
          }
          if (autoCounter == 1) {
            if (!toplimitswitch.get()) {
              m_elevator.set(-0.8 / 2);
            } else if (toplimitswitch.get()) {
              m_elevator.set(0.0);
              timer.reset();
              autoCounter++;
            } else {
              autoCounter = 8;
              System.out.println("error");
            }
          }
          if (autoCounter == 2) {
            if (timer.get() < 0.1) {
              m_elevator.set(0.3 / 2); //high
            } else {
              m_elevator.set(0.0);
              timer.reset();
              autoCounter++;
            }
          }
          if (autoCounter == 3) {
            if (timer.get() < 2) {
              ROTATION.set(0.7);
            } else {
              ROTATION.set(0.0);
              timer.reset();
              autoCounter++;
            }
          }
          if (autoCounter == 4) {
            if (timer.get() < 0.5) {
              INTAKE.set(-0.9);
            } else {
              INTAKE.set(0.0);
              timer.reset();
              autoCounter++;
            }
          }
          if (autoCounter == 5) {
            if (timer.get() < 0.7) {
              ROTATION.set(-0.6);
            } else {
              ROTATION.set(0.0);
              timer.reset();
              autoCounter++;
            }
          }
          if (autoCounter == 6) {
            if (timer.get() < 3.5) {
              m_elevator.set(0.3 / 2);
            } else {
              m_elevator.set(0.0);
              timer.reset();
              autoCounter++;
            }
          } //end of high
          if (autoCounter == 7) {
            if (timer.get() < 3.5) {
              ROBOT_DRIVE.arcadeDrive(0.60, 0.0);
            } else {
              ROBOT_DRIVE.arcadeDrive(0.0, 0.0);
              timer.reset();
              autoCounter++;
            }
          }
          break;

        case kDefaultAuto:
          if (autoCounter == 0) {
            if (timer.get() < 0.3) {
              ROTATION.set(-0.5);
            } else {
              ROTATION.set(0.0);
              timer.reset();
              autoCounter++;
            }
          }
          if (autoCounter == 1) {
            if (!toplimitswitch.get()) {
              m_elevator.set(-1.0 / 2);
            } else if (toplimitswitch.get()) {
              m_elevator.set(0.0);
              timer.reset();
              autoCounter++;
            } else {
              autoCounter = 8;
              System.out.println("error");
            }
          }
          if (autoCounter == 2) {
            if (timer.get() < 0.1) {
              m_elevator.set(0.3 / 2); //high
            } else {
              m_elevator.set(0.0);
              timer.reset();
              autoCounter++;
            }
          }
          if (autoCounter == 3) {
            if (timer.get() < 0.7) {
              ROTATION.set(0.9);
            } else {
              ROTATION.set(0.0);
              timer.reset();
              autoCounter++;
            }
          }
          if (autoCounter == 4) {
            if (timer.get() < 0.5) {
              INTAKE.set(-0.9);
            } else {
              INTAKE.set(0.0);
              timer.reset();
              autoCounter++;
            }
          }
          if (autoCounter == 5) {
            if (timer.get() < 0.7) {
              ROTATION.set(-0.6);
            } else {
              ROTATION.set(0.0);
              timer.reset();
              autoCounter++;
            }
          }
          if (autoCounter == 6) {
            if (timer.get() < 1.5) {
              m_elevator.set(0.8 / 2);
            } else {
              m_elevator.set(0.0);
              timer.reset();
              autoCounter++;
            }
          } //end of high
          if (autoCounter == 7) {
            if (timer.get() < 2.0) {
              ROBOT_DRIVE.arcadeDrive(0.65, 0.0);
            } else if (encoderA.getDistance() > -1240) {
              ROBOT_DRIVE.arcadeDrive(0.55, 0.0);
            } else {
              ROBOT_DRIVE.arcadeDrive(0.0, 0.0);
              timer.reset();
              autoCounter++;
            }
          }
          break;

        default:
          // Put default auto code here
          break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    ROTATION.set(0.0);
    INTAKE.set(0.0);
    m_elevator.set(0.0);
    ROBOT_DRIVE.arcadeDrive(0.0, 0.0);
    timer.reset();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");

    //read values periodically
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);

    //post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
    SmartDashboard.putNumber("Roll", gyro.getRoll());
    SmartDashboard.putNumber("Pitch", gyro.getPitch());
    SmartDashboard.putNumber("Yaw", gyro.getYaw());


    SmartDashboard.putNumber("Elevator Encoder", m_elevatorEncoder.getPosition());
    SmartDashboard.putBoolean("Limit Switch", toplimitswitch.get());
    SmartDashboard.putNumber("EncoderA", encoderA.getDistance());
    // SmartDashboard.putNumber("EncoderB", encoderB.getDistance());


    // elevator below

    toggle = toplimitswitch.get(); // current position of limit switch
    
    if (toplimitswitch.get()) { //resets encoder value and accounts for drift
      m_elevatorEncoder.setPosition(-219);
    }
    if (JOYSTICK2.getRawButton(1)) { //mid button -- brings elevator to the mid level
      mid = true;
      elevatortimer.reset();
      elevatortimer.start();
    }

    // in a function
    if (mid) {
      elevator(); 
    }
    // if (mid) { // if mid is pressed set to mid 
    //   sensorposition = -m_elevatorEncoder.getPosition();
    //   error = setpoint-sensorposition;
    //   dt = Timer.getFPGATimestamp() - lastTimestamp;
    //   if (Math.abs(error) < iLimit) {
    //     errorSum += error*dt;
    //   }
  
    //   errorRate = (error - lasterror) / dt;
  
    //   lastTimestamp = Timer.getFPGATimestamp();
    //   lasterror = error;
      
    //   m_elevator.set(-kp*error/2);
    //   if (error < 5) { //close to mid done process
    //     mid = false;
    //     m_elevator.set(0.0);
    //   }
    // }
    SmartDashboard.putNumber("error", error); // if limit switch == true and trying to go up, it will go down
    if (JOYSTICK2.getRawButtonPressed(4) && !up) { // elevator down
      if (toggle) {
        timer.reset();
        timer.start();
        if (timer.get() < 0.3) {
          m_elevator.set(0.2 / 2);
        }
        System.out.println("Yes");
        up = false;
      } else { // elevator up
        m_elevator.set(-1.2 / 2);
        System.out.println("no");
        up = true;
      }
    }
    if (up) { //if up== true limit switch true, turn off elevator and let slide little
      if (toggle) {
        m_elevator.set(0.0);
        timer.reset();
        timer.start();
        if (timer.get() < 0.3) { // go down for a little
          m_elevator.set(0.2 / 2);
        }
        up = false;
      }
    }

    intake();

    if (JOYSTICK2.getRawButtonReleased(4)) { // release up then turn off elevator
      m_elevator.set(0.0);
      up = false;
    }

    if (JOYSTICK2.getRawButtonPressed(2)) { // elevator goes down
      System.out.println("goofy");
      m_elevator.set(0.6 / 2);

    }
    if (JOYSTICK2.getRawButtonReleased(2)) { //elevator stops
      m_elevator.set(0.0);
    }

    if (JOYSTICK2.getRawButtonPressed(7)) { // intake rotation down
      ROTATION.set(-0.55);
    }
    if (JOYSTICK2.getRawButtonReleased(7)) { // intake release stall to assist in decending
      ROTATION.set(0.15);
    }
    if (JOYSTICK2.getRawButtonPressed(5)) { // intake rotation up
      ROTATION.set(0.95);
    }
    if (JOYSTICK2.getRawButtonReleased(5)) { // intake release stall to assist in decending 
      ROTATION.set(0.15);
    }
    if (JOYSTICK2.getRawButton(3)) { // intake hard turn off -- kill
      INTAKE.set(0.0);
    }
    
    if (JOYSTICK.getRawButton(4)) {
      align = true;
    }

    if (JOYSTICK.getRawButton(3)) {
      turn = true;
      turnsetpoint = gyro.getYaw() + 90;
    }

    if (align) {
      error = x;

      if (error > 0) {
        ROBOT_DRIVE.arcadeDrive(0.45, 0.0);
      } else if (error < 0) {
        ROBOT_DRIVE.arcadeDrive(-0.45, 0.0);
      }

      if (-4 < error && error < 0) {
        align = false;
        ROBOT_DRIVE.arcadeDrive(0.0, 0.0);
        // turn = true;
      }

    }

    if (turn) {
      error = turnsetpoint - gyro.getYaw();
      ROBOT_DRIVE.arcadeDrive(0.0, -0.007 * error);
      SmartDashboard.putNumber("turn value", -0.007 * error);
      if (error < 5) {
        turn = false;
        ROBOT_DRIVE.arcadeDrive(0.0, 0.0);
      }
    }

    if (JOYSTICK.getRawButton(6)) {
      turn = false;
      align = false;
    }

    if (JOYSTICK.getRawButton(1)) { //slow drive train
      slowdrive = 0.50;
      slowturn = 0.80;
    }
    if (JOYSTICK.getRawButton(2)) { // go back to normal
      slowdrive = 1.0;
      slowturn = 1.0;
    }

    if (!turn && !align) {
      ROBOT_DRIVE.arcadeDrive(JOYSTICK.getRawAxis(1) * slowdrive, -JOYSTICK.getRawAxis(4) * 0.7 * slowturn); // set everything to drive train
    }
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() { }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() { }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    autoCounter = 0;
    timer.reset();
    timer.start();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    if (gyro.getPitch() > 5) {
      ROBOT_DRIVE.arcadeDrive(0.45, 0.0);
    } else {
      ROBOT_DRIVE.arcadeDrive(0.0, 0.0);
    }
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() { }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() { }

  public void intake() {
    if (JOYSTICK2.getRawButtonPressed(6)) { 
      cubein(); 
    } 

    if (JOYSTICK2.getRawButtonPressed(8)) { 
      cubeout(); 
    }

    if (JOYSTICK2.getRawButtonReleased(6)) { 
      cubestall(); 
    }

    if (JOYSTICK2.getRawButtonReleased(8)) { 
      conestall(); 
    }
  }

  public void dropintake() {
    if (intaketimer.get() < 0.3) {
      ROTATION.set(-0.5);
    } else {
      ROTATION.set(0.0);
    }
  }

  public void cubestall() {
    INTAKE.set(0.2);
  }

  public void conestall() {
    INTAKE.set(-0.2);
  }

  public void elevator() {
    
    m_elevator.set(0.3 / elevatortimer.get());

    SmartDashboard.putNumber("speed of elevator", kp * error / 2);
    if (toplimitswitch.get()) { //close to mid done process
      mid = false;
      m_elevator.set(0.0);
    }
  }

  public void cubeout() {
    INTAKE.set(-0.5);
  }

  public void cubein() {
    INTAKE.set(0.4);
  }

  public void intakeoff() {
    INTAKE.set(0.0);
  }

  // group all cube stuff into methods -- split into intake/outake
  // set global with meaning variables
  // split tasks (move cube to mid --> dropintake - pull cube - stop move - lift to mid - dro cube - drop mid - drop intake )
  // resetelevator()
  // reset intake()
  // intake cube (speed, button)
  // outake cube (speed, button)
  // lift (height: float)
}
