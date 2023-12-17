package org.bitbuckets;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

public class Robot extends TimedRobot {
    WPI_TalonFX talonFX1;
    WPI_TalonFX talonFX2;


    SimpleMotorFeedforward ff;
    PIDController pid;
    DifferentialDrive differentialDrive;
    DifferentialDriveKinematics kinematics;
    ChassisSpeeds chassisSpeeds;
    Joystick joystick;

    double ks = 0.65292;
    double kv = 2.3053;
    double kp = 0;
    double ki = 0;
    double kd = 0;


    @Override
    public void robotInit() {
        super.robotInit();
        talonFX1 = new WPI_TalonFX(1);
        talonFX2 = new WPI_TalonFX(17);
        differentialDrive = new DifferentialDrive(talonFX1, talonFX2);
        joystick = new Joystick(0);
        kinematics = new DifferentialDriveKinematics(12);
        ff = new SimpleMotorFeedforward(ks,kv,0.37626);
        pid = new PIDController(kp, ki, kd);
        chassisSpeeds = new ChassisSpeeds();



    }

    @Override
    public void teleopPeriodic() {
        super.teleopPeriodic();

        //SETTING WHEELS SPEEDS TO JOYSTICK AXES
        double forwardSpeed = joystick.getRawAxis(0);
        double rotationSpeed = joystick.getRawAxis(4);

        //GETTING WHEEL SPEEDS FROM CHASSIS SPEEDS
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(forwardSpeed, 0.0, rotationSpeed, new Rotation2d());
        double leftSpeeds = kinematics.toWheelSpeeds(speeds).leftMetersPerSecond;
        double rightSpeeds = kinematics.toWheelSpeeds(speeds).rightMetersPerSecond;

        //CALC FF FROM WHEELS SPEEDS
        double leftFF = ff.calculate(leftSpeeds);
        double rightFF = ff.calculate(rightSpeeds);

        //SET THE CALC VOLTAGE FOR EACH MOTOR CONTROLLER
        talonFX1.setVoltage(leftFF);
        talonFX2.setVoltage(rightFF);





    }
}