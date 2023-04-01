// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.platform.can.AutocacheState;
import com.kauailabs.navx.frc.AHRS;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  int state=0;
  int debounceCount=0;
  double onChargeStationDegree = 13.0;
  double debounceTime = 0.2;
  double robotSpeedSlow=0.325;
  double robotSpeedFast=0.65;
  double levelDegree=6.0;
  boolean toggle = false;
  boolean up = false;
  int autoCounter=0;
  final double kp=0.01;
  final double ki=0.00;
  final double iLimit = 2;
  final double kd=0.00;
  double dt = 0.0;
  double setpoint = 116.0;
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
  private static final String kDefaultAuto = "Cube + Engage";
  private static final String kCustomAuto = "Cube + Mobility";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private final Timer timer = new Timer();

  final Joystick JOYSTICK = new Joystick(0);
  final Joystick JOYSTICK2 = new Joystick(1);

  
  final WPI_VictorSPX FRONT_LEFT = new WPI_VictorSPX(4);
  final WPI_VictorSPX FRONT_RIGHT = new WPI_VictorSPX(6);
  final WPI_VictorSPX BACK_LEFT = new WPI_VictorSPX(12);
  final WPI_VictorSPX BACK_RIGHT = new WPI_VictorSPX(5);
  final WPI_VictorSPX ROTATION = new WPI_VictorSPX(13);


  final MotorControllerGroup LEFT_DRIVE = new MotorControllerGroup(FRONT_LEFT, BACK_LEFT);
  final MotorControllerGroup RIGHT_DRIVE = new MotorControllerGroup(FRONT_RIGHT, BACK_RIGHT);
  final Encoder encoderA = new Encoder(0, 1);
  final Encoder encoderB = new Encoder(2, 3);

  final DifferentialDrive ROBOT_DRIVE = new DifferentialDrive(LEFT_DRIVE, RIGHT_DRIVE);

  final CANSparkMax m_leftElevator = new CANSparkMax(1, MotorType.kBrushless);
  final CANSparkMax m_rightElevator = new CANSparkMax(2, MotorType.kBrushless);
  final CANSparkMax INTAKE = new CANSparkMax(3, MotorType.kBrushless);

  final RelativeEncoder m_elevatorEncoder = m_rightElevator.getEncoder();


  final MotorControllerGroup m_elevator = new MotorControllerGroup(m_leftElevator, m_rightElevator);
  final AHRS gyro = new AHRS(SPI.Port.kMXP);
  final DigitalInput toplimitswitch = new DigitalInput(4);

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_rightElevator.setInverted(true);
    RIGHT_DRIVE.setInverted(true);
    m_chooser.setDefaultOption("Cube + Engage", kDefaultAuto);
    m_chooser.addOption("Cube + Mobility", kCustomAuto);
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
    state=0;
    debounceCount=0;
    autoCounter=0;
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
    timer.reset();
    timer.start();
    encoderA.reset();
    encoderB.reset();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:

        if (autoCounter==0) {
          if (timer.get() < 0.3) {
            ROTATION.set(-0.5);
          }
          else {
            ROTATION.set(0.0);
            timer.reset();
            autoCounter++;
          }
        }
        if (autoCounter==1) {
          if (toplimitswitch.get() == false) {
            m_elevator.set(-0.8);
          }
          else if (toplimitswitch.get() == true) {
            m_elevator.set(0.0);
            timer.reset();
            autoCounter++;
          }
          else {
            autoCounter=8;
            System.out.println("error");
          }
        }
        if (autoCounter==2) {
          if (timer.get() < 0.1) {
            m_elevator.set(0.3);
          }
          else {
            m_elevator.set(0.0);
            timer.reset();
            autoCounter++;
          }
        }
        if (autoCounter==3) {
          if (timer.get() < 2.7) {
            ROTATION.set(0.7);
          }
          else {
            ROTATION.set(0.0);
            timer.reset();
            autoCounter++;
          }
        }
        if (autoCounter==4) {
          if (timer.get() < 1) {
            INTAKE.set(-0.6);
          }
          else {
            INTAKE.set(0.0);
            timer.reset();
            autoCounter++;
          }
        }
        if (autoCounter==5) {
          if (timer.get() < 0.7) {
            ROTATION.set(-0.6);
          }
          else {
            ROTATION.set(0.0);
            timer.reset();
            autoCounter++;
          }
        }
        if (autoCounter==6) {
          if (timer.get() < 1.4) {
            m_elevator.set(0.7);
          }
          else {
            m_elevator.set(0.0);
            timer.reset();
            autoCounter++;
          }
        }
        if (autoCounter==7) {
          if (timer.get() < 3) {
            ROBOT_DRIVE.arcadeDrive(0.6, 0.0);
          }
          else if (encoderA.getDistance() < 2800) {
            ROBOT_DRIVE.arcadeDrive(0.55, 0.0);
          }
          else {
            ROBOT_DRIVE.arcadeDrive(0.0, 0.0);
            autoCounter++;
            encoderA.reset();
          }
        }
        if (autoCounter==8) {
          if (encoderA.getDistance() < 430) {
            ROBOT_DRIVE.arcadeDrive(0.0, 0.5);
          }
          else {
            ROBOT_DRIVE.arcadeDrive(0.0, 0.0);
            autoCounter++;
          }
        }
        break;
      case kDefaultAuto:

        if (autoCounter==0) {
          if (timer.get() < 0.3) {
            ROTATION.set(-0.5);
          }
          else {
            ROTATION.set(0.0);
            timer.reset();
            autoCounter++;
          }
        }
        if (autoCounter==1) {
          if (toplimitswitch.get() == false) {
            m_elevator.set(-0.8);
          }
          else if (toplimitswitch.get() == true) {
            m_elevator.set(0.0);
            timer.reset();
            autoCounter++;
          }
          else {
            autoCounter=8;
            System.out.println("error");
          }
        }
        if (autoCounter==2) {
          if (timer.get() < 0.1) {
            m_elevator.set(0.3);
          }
          else {
            m_elevator.set(0.0);
            timer.reset();
            autoCounter++;
          }
        }
        if (autoCounter==3) {
          if (timer.get() < 2.7) {
            ROTATION.set(0.7);
          }
          else {
            ROTATION.set(0.0);
            timer.reset();
            autoCounter++;
          }
        }
        if (autoCounter==4) {
          if (timer.get() < 0.5) {
            INTAKE.set(-0.6);
          }
          else {
            INTAKE.set(0.0);
            timer.reset();
            autoCounter++;
          }
        }
        if (autoCounter==5) {
          if (timer.get() < 0.7) {
            ROTATION.set(-0.6);
          }
          else {
            ROTATION.set(0.0);
            timer.reset();
            autoCounter++;
          }
        }
        if (autoCounter==6) {
          if (timer.get() < 1.4) {
            m_elevator.set(0.7);
          }
          else {
            m_elevator.set(0.0);
            timer.reset();
            autoCounter++;
          }
        }
        if (autoCounter==7) {
          if (timer.get() < 3) {
            ROBOT_DRIVE.arcadeDrive(0.6, 0.0);
          }
          else if (encoderA.getDistance() < 1210) { //1240
            ROBOT_DRIVE.arcadeDrive(0.65, 0.0);
          }
          else {
            ROBOT_DRIVE.arcadeDrive(0.0, 0.0);
            autoCounter++;
          }
        }
      default:
        // Put default auto code here
        break;
    }
  }

  public int secondsToTicks(double time){
    return (int)(time*50);
  }
  public double pidEngage(AHRS gyro) {
    sensorposition = -gyro.getPitch();
    System.out.println(sensorposition);
    error = sensorposition-setpoint;
    System.out.println(error);
    dt = Timer.getFPGATimestamp() - lastTimestamp;
    if (Math.abs(error) < iLimit) {
      errorSum += error*dt;
    }

    errorRate = (error - lasterror) / dt;

    lastTimestamp = Timer.getFPGATimestamp();
    lasterror = error;
    return (kp*error + ki*errorSum + kd*errorRate);
  }
  public double gyroEngage(AHRS gyro){
    switch (state){
            //drive forwards to approach station, exit when tilt is detected
            case 0:
                //Once docked, start incrementing this variable
                if(-gyro.getPitch() > onChargeStationDegree){
                    debounceCount++;
                }
                //If its been docked for 200ms, change states
                if(debounceCount > secondsToTicks(debounceTime)){
                    state = 1;
                    debounceCount = 0;
                    return robotSpeedSlow;
                }
                return robotSpeedFast;
            //driving up charge station, drive slower, stopping when level
            case 1:
                //Once starts to level (levelDegree=6), increase debount count
                if (-gyro.getPitch() < levelDegree){
                    debounceCount++; 
                }
                //If been level for 200ms, change state
                if(debounceCount > secondsToTicks(debounceTime)){
                    state = 3;
                    debounceCount = 0;
                    return 0;
                }
                return robotSpeedSlow;
            //on charge station, stop motors and wait for end of auto. Sort of like autocorrection
            case 2:
                //If the tilt is less than HALF the level degree, start increasing debounce count
                if(Math.abs(-gyro.getPitch()) <= levelDegree){
                    debounceCount++;
                }
                //If 200ms passed since half level degree, go to state 4 and robot will stop
                if(debounceCount>secondsToTicks(debounceTime)){
                    state = 3;
                    debounceCount = 0;
                    return 0;
                }
                //If the roll is gerater than the level degree (robot is falling backwards, then VERY slowly go up)
                if(-gyro.getPitch() >= levelDegree) {
                    debounceCount=0;
                    return 0.53;
                //If instead the roll is less than the negative level degree (robot is falling forwards), then very slowly up BACKWARDS on station
                } else if(-gyro.getPitch() <= -levelDegree) {
                    debounceCount=0;
                    return -0.50;
                }
            //Useless should remove but keep just in case
            case 3:
                  return 0;
        }
        return 0;

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
    SmartDashboard.putNumber("Roll", gyro.getRoll());
    SmartDashboard.putNumber("Pitch", gyro.getPitch());
    SmartDashboard.putNumber("Yaw", gyro.getYaw());


    SmartDashboard.putNumber("Elevator Encoder", m_elevatorEncoder.getPosition());
    SmartDashboard.putBoolean("Limit Switch", toplimitswitch.get());
    SmartDashboard.putNumber("EncoderA", encoderA.getDistance());
    if (JOYSTICK2.getRawButtonPressed(6)) {
      INTAKE.set(0.4);
    } 
    if (JOYSTICK2.getRawButtonReleased(6)) {
      INTAKE.set(0.2);
    }
    if (JOYSTICK2.getRawButtonPressed(8)) {
      INTAKE.set(-0.5);
    }
    if (JOYSTICK2.getRawButtonReleased(8)) {
      INTAKE.set(-0.2);
    }

    // toggle = toplimitswitch.get();

    // if (JOYSTICK2.getRawButton(1)) {
    //   m_elevatorEncoder.setPosition(0.0);
    //   mid = true;
    // }

    // if (mid) {
    //   sensorposition = -m_elevatorEncoder.getPosition();
    //   error = setpoint-sensorposition;
    //   dt = Timer.getFPGATimestamp() - lastTimestamp;
    //   if (Math.abs(error) < iLimit) {
    //     errorSum += error*dt;
    //   }
  
    //   errorRate = (error - lasterror) / dt;
  
    //   lastTimestamp = Timer.getFPGATimestamp();
    //   lasterror = error;
      
    //   m_elevator.set(-kp*error);
    //   if (error < 5) {
    //     mid = false;
    //     m_elevator.set(0.0);
    //   }
    // }
    // SmartDashboard.putNumber("error", error);
    // if (JOYSTICK2.getRawButtonPressed(4) && up == false) {
    //   if (toggle) {
    //     timer.reset();
    //     timer.start();
    //     if (timer.get() < 0.3) {
    //       m_elevator.set(0.2);
    //     }
    //     System.out.println("Yes");
    //     up = false;
    //   }
    //   else {
    //     m_elevator.set(-0.8);
    //     System.out.println("no");
    //     up = true;
    //   }
    // }
    // if (up) {
    //   if (toggle) {
    //     m_elevator.set(0.0);
    //     timer.reset();
    //     timer.start();
    //     if (timer.get() < 0.3) {
    //       m_elevator.set(0.2);
    //     }
    //     up = false;
    //   }
    // }

    // if (JOYSTICK2.getRawButtonReleased(4)) {
    //   m_elevator.set(0.0);
    //   up = false;
    // }

    // if (JOYSTICK2.getRawButtonPressed(2)) {
    //   System.out.println("goofy");
    //   m_elevator.set(0.8);
    // }
    // if (JOYSTICK2.getRawButtonReleased(2)) {
    //   m_elevator.set(0.0);
    // }







    toggle = toplimitswitch.get();
    
    if (toplimitswitch.get()) {
      m_elevatorEncoder.setPosition(-219);
    }
    if (JOYSTICK2.getRawButton(1)) {
      m_elevatorEncoder.setPosition(0.0);
      mid = true;
    }

    if (mid) {
      sensorposition = -m_elevatorEncoder.getPosition();
      error = setpoint-sensorposition;
      dt = Timer.getFPGATimestamp() - lastTimestamp;
      if (Math.abs(error) < iLimit) {
        errorSum += error*dt;
      }
  
      errorRate = (error - lasterror) / dt;
  
      lastTimestamp = Timer.getFPGATimestamp();
      lasterror = error;
      
      // m_elevator.set(-kp*error);
      m_rightElevator.set(-kp*error);
      m_leftElevator.set(-kp*error*0.333333333333333333333333333);
      if (error < 5) {
        mid = false;
        m_elevator.set(0.0);
      }
    }
    SmartDashboard.putNumber("error", error);
    if (JOYSTICK2.getRawButtonPressed(4) && up == false) {
      if (toggle) {
        timer.reset();
        timer.start();
        if (timer.get() < 0.3) {
          // m_elevator.set(0.2);
          m_rightElevator.set(0.3);
          m_leftElevator.set(0.3*0.3333333333333333333333333333333333);
        }
        System.out.println("Yes");
        up = false;
      }
      else {
        // m_elevator.set(-0.8);
        m_rightElevator.set(-0.9);
        m_leftElevator.set(-0.9*0.3333333333333333333333333333333333333333);
        System.out.println("no");
        up = true;
      }
    }
    if (up) {
      if (toggle) {
        m_elevator.set(0.0);
        timer.reset();
        timer.start();
        if (timer.get() < 0.3) {
          // m_elevator.set(0.2);
          m_rightElevator.set(0.3);
          m_leftElevator.set(0.3*0.3333333333333333333333333333333333);
        }
        up = false;
      }
    }

    if (JOYSTICK2.getRawButtonReleased(4)) {
      m_elevator.set(0.0);
      up = false;
    }

    if (JOYSTICK2.getRawButtonPressed(2)) {
      System.out.println("goofy");
      // m_elevator.set(0.8);
      m_rightElevator.set(0.7);
      m_leftElevator.set(0.7*0.3333333333333333333333333333333333333);

    }
    if (JOYSTICK2.getRawButtonReleased(2)) {
      m_elevator.set(0.0);
    }

    if (JOYSTICK2.getRawButtonPressed(7)) {
      ROTATION.set(-0.6);
    }
    if (JOYSTICK2.getRawButtonReleased(7)) {
      ROTATION.set(0.15);
    }
    if (JOYSTICK2.getRawButtonPressed(5)) {
      ROTATION.set(0.85);
    }
    if (JOYSTICK2.getRawButtonReleased(5)) {
      ROTATION.set(0.15);
    }
    if (JOYSTICK2.getRawButton(3)) {
      INTAKE.set(0.0);
    }
    
    if (JOYSTICK.getRawButton(1)) {
      slowdrive = 0.50;
      slowturn = 0.80;
    }
    if (JOYSTICK.getRawButton(2)) {
      slowdrive = 1.0;
      slowturn = 1.0;
    }
    ROBOT_DRIVE.arcadeDrive(JOYSTICK.getRawAxis(1)*slowdrive, -JOYSTICK.getRawAxis(4)*0.7*slowturn);
    // if (JOYSTICK.getRawButton(1)) {
    //   gyroEngage(gyro);
    // //   // if (gyro.getPitch() < -4) {
    // //   //   ROBOT_DRIVE.arcadeDrive(-0.4, 0.0);
    // //   // }
    // //   ROBOT_DRIVE.arcadeDrive(pidEngage(gyro), 0.0);
    // }
    System.out.println(gyro.getPitch()-lastpitch);
    lastpitch = gyro.getPitch();
    
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    autoCounter=0;
    timer.reset();
    timer.start();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    // ROBOT_DRIVE.arcadeDrive(gyroEngage(gyro), 0.0);
    toggle = toplimitswitch.get();

    if (JOYSTICK2.getRawButton(1)) {
      m_elevatorEncoder.setPosition(0.0);
      mid = true;
    }

    if (mid) {
      sensorposition = -m_elevatorEncoder.getPosition();
      error = setpoint-sensorposition;
      dt = Timer.getFPGATimestamp() - lastTimestamp;
      if (Math.abs(error) < iLimit) {
        errorSum += error*dt;
      }
  
      errorRate = (error - lasterror) / dt;
  
      lastTimestamp = Timer.getFPGATimestamp();
      lasterror = error;
      
      // m_elevator.set(-kp*error);
      m_rightElevator.set(-kp*error);
      m_leftElevator.set(-kp*error*0.333333333333333333333333333);
      if (error < 5) {
        mid = false;
        m_elevator.set(0.0);
      }
    }
    SmartDashboard.putNumber("error", error);
    if (JOYSTICK2.getRawButtonPressed(4) && up == false) {
      if (toggle) {
        timer.reset();
        timer.start();
        if (timer.get() < 0.3) {
          // m_elevator.set(0.2);
          m_rightElevator.set(0.2);
          m_leftElevator.set(0.2*0.3333333333333333333333333333333333);
        }
        System.out.println("Yes");
        up = false;
      }
      else {
        // m_elevator.set(-0.8);
        m_rightElevator.set(-0.8);
        m_leftElevator.set(-0.8*0.3333333333333333333333333333333333333333);
        System.out.println("no");
        up = true;
      }
    }
    if (up) {
      if (toggle) {
        m_elevator.set(0.0);
        timer.reset();
        timer.start();
        if (timer.get() < 0.3) {
          // m_elevator.set(0.2);
          m_rightElevator.set(0.2);
          m_leftElevator.set(0.2*0.3333333333333333333333333333333333);
        }
        up = false;
      }
    }

    if (JOYSTICK2.getRawButtonReleased(4)) {
      m_elevator.set(0.0);
      up = false;
    }

    if (JOYSTICK2.getRawButtonPressed(2)) {
      System.out.println("goofy");
      // m_elevator.set(0.8);
      m_rightElevator.set(0.8);
      m_leftElevator.set(0.8*0.3333333333333333333333333333333333333);

    }
    if (JOYSTICK2.getRawButtonReleased(2)) {
      m_elevator.set(0.0);
    }

  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
