// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;


public class DriveSubsystem extends SubsystemBase {
  // These represent our regular encoder objects, which we would
  // create to use on a real robot.
  private Encoder m_leftEncoder = new Encoder(0, 1);
  private Encoder m_rightEncoder = new Encoder(2, 3);

  // These are our EncoderSim objects, which we will only use in
  // simulation. However, you do not need to comment out these
  // declarations when you are deploying code to the roboRIO.
  private EncoderSim m_leftEncoderSim = new EncoderSim(m_leftEncoder);
  private EncoderSim m_rightEncoderSim = new EncoderSim(m_rightEncoder);

  private PWMSparkMax m_leftMotor = new PWMSparkMax(0);
  private PWMSparkMax m_rightMotor = new PWMSparkMax(1);

  ///Timer
  private final Timer m_timer = new Timer();
  
  // Create our gyro object like we would on a real robot.
  private AnalogGyro m_gyro = new AnalogGyro(1);

  // Create the simulated gyro object, used for setting the gyro
  // angle. Like EncoderSim, this does not need to be commented out
  // when deploying code to the roboRIO.
  private AnalogGyroSim m_gyroSim = new AnalogGyroSim(m_gyro);

  // Create our feedforward gain constants (from the identification tool)
  static final double KvLinear = 1.98;
  static final double KaLinear = 0.2;
  static final double KvAngular = 1.5;
  static final double KaAngular = 0.3;

  // Create the simulation model of our drivetrain.
  DifferentialDrivetrainSim m_driveSim = new DifferentialDrivetrainSim(
  DCMotor.getNEO(2),       // 2 NEO motors on each side of the drivetrain.
  7.29,                    // 7.29:1 gearing reduction.
  7.5,                     // MOI of 7.5 kg m^2 (from CAD model).
  60.0,                    // The mass of the robot is 60 kg.
  Units.inchesToMeters(3), // The robot uses 3" radius wheels.
  0.7112,                  // The track width is 0.7112 meters.

  // The standard deviations for measurement noise:
  // x and y:          0.001 m
  // heading:          0.001 rad
  // l and r velocity: 0.1   m/s
  // l and r position: 0.005 m
  VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005));

  private Field2d m_field = new Field2d();
  

  DifferentialDrive differentialDrive = null;

  public DriveSubsystem() {
    m_leftEncoder.setDistancePerPulse(2 * Math.PI * 3 / 4096);
    m_rightEncoder.setDistancePerPulse(2 * Math.PI * 3 / 4096);
    SmartDashboard.putData("Field", m_field);

    DifferentialDrive differentialDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);
  }

  XboxController m_driverController = new XboxController(0);

  
  ///Move/drive Command (practicing commands rlly helped!) (1)
  public void movedrive() {
    ///just trying to make robot move
    differentialDrive.arcadeDrive(1, 45);
  }

  public InstantCommand driveCmd(){
    return new InstantCommand(() -> this.movedrive());
  }

  ///ChangingSpeed (2)
  public void setspeed() {
    if (m_driverController.getAButtonPressed()) {
      SmartDashboard.putString("Log","setting speed to 0.25");
      m_leftMotor.set(0.25);
      m_rightMotor.set(0.25);
    }
    else if (m_driverController.getBButtonPressed()) {
      SmartDashboard.putString("Log","setting speed to 0.5");
      m_leftMotor.set(0.5);
      m_rightMotor.set(0.5);
    }
    else if (m_driverController.getXButtonPressed()) {
      SmartDashboard.putString("Log","setting speed to 0.75");
      m_leftMotor.set(0.75);
      m_rightMotor.set(0.75);
    }
    else if (m_driverController.getYButtonPressed()) {
      SmartDashboard.putString("Log","setting speed to 1");
      m_leftMotor.set(0.1);
      m_rightMotor.set(0.1);
    }
    else {
      SmartDashboard.putString("Log","speed is 0");
      m_leftMotor.set(0);
      m_rightMotor.set(0);
    }
  }

  ///move forward for 2 seconds, wait 1 second, then move backward 2 seconds (3)
  ///Using logic where both motors inverted false makes the robot move forward while both motors inverted true makes robot move backward
  public void waitCmdinitalize() {
    m_timer.restart();
    if (m_timer.get() < 2) {
      ///go forward 2 seconds (using logic above)
      m_rightMotor.setInverted(false);
      m_leftMotor.setInverted(false);
      differentialDrive.arcadeDrive(0.5, 0);
    } else if (m_timer.get() >= 2 && m_timer.get() < 3) {
      ///using wait command to wait
      new WaitCommand(1);
    } else if (m_timer.get() >= 3 && m_timer.get() < 5) {
      ///go backward 2 seconds (using logic above)
      m_rightMotor.setInverted(true);
      m_leftMotor.setInverted(true);
      differentialDrive.arcadeDrive(-0.5, 0);
    } else {
      ///just decided to stop the robot if it doesn't meet the other requirements above
      differentialDrive.stopMotor();
    }
  }

  public InstantCommand waitCmd(){
    return new InstantCommand(() -> this.waitCmdinitalize());
  }



  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    // Set the inputs to the system. Note that we need to convert
    // the [-1, 1] PWM signal to voltage by multiplying it by the
    // robot controller voltage.
    m_driveSim.setInputs(m_leftMotor.get() * RobotController.getInputVoltage(),
    m_rightMotor.get() * RobotController.getInputVoltage());

    // Advance the model by 20 ms. Note that if you are running this
    // subsystem in a separate thread or have changed the nominal timestep
    // of TimedRobot, this value needs to match it.
    m_driveSim.update(0.02);

    // Update all of our sensors.
    m_leftEncoderSim.setDistance(m_driveSim.getLeftPositionMeters());
    m_leftEncoderSim.setRate(m_driveSim.getLeftVelocityMetersPerSecond());
    m_rightEncoderSim.setDistance(m_driveSim.getRightPositionMeters());
    m_rightEncoderSim.setRate(m_driveSim.getRightVelocityMetersPerSecond());
    m_gyroSim.setAngle(-m_driveSim.getHeading().getDegrees());

    // This will get the simulated sensor readings that we set
    // in the previous article while in simulation, but will use
    // real values on the robot itself.
    m_odometry.update(m_gyro.getRotation2d(),
                    m_leftEncoder.getDistance(),
                    m_rightEncoder.getDistance());
    m_field.setRobotPose(m_odometry.getPoseMeters());
  }

}
