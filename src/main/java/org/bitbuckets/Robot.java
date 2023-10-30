package org.bitbuckets;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.numbers.N7;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {

    static final float GEARING_ROT = 150;
    static final float MOI_JKG = 30;
    static final float MASS_KG = 5;
    static final float TRACK_WIDTH_M = 1;
    static final float WHEEL_RADIUS_M = 0.5f;
    static final Vector<N7> STD_DEVS = VecBuilder.fill(0,0,0,0,0,0,0);
    static final int PORT = 0;

    XboxController driverInput = new XboxController(1);

    DifferentialDrivetrainSim sim = new DifferentialDrivetrainSim(
            DCMotor.getNEO(1),
            GEARING_ROT,
            MOI_JKG,
            MASS_KG,
            WHEEL_RADIUS_M,
            TRACK_WIDTH_M,
            STD_DEVS
    );

    DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(TRACK_WIDTH_M);

    @Override
    public void robotInit() {


    }

    @Override
    public void robotPeriodic() {

        double leftJoystickX = driverInput.getLeftX();
        double leftJoystickY = driverInput.getLeftY();
        double rightJoystickX = driverInput.getRightX();

        ChassisSpeeds speed = new ChassisSpeeds(leftJoystickX,leftJoystickY,rightJoystickX);
        DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speed);
        SimpleMotorFeedforward ff = new SimpleMotorFeedforward(1,1,1);

        double leftVoltageDesired = ff.calculate(wheelSpeeds.leftMetersPerSecond);
        double rightVoltageDesired = ff.calculate(wheelSpeeds.rightMetersPerSecond);



        sim.setInputs()





        //logging
        sim.update(0.02);

        Pose2d estimatedPose = sim.getPose();
        double[] poseArray = new double[]{
                estimatedPose.getX(),
                estimatedPose.getY(),
                estimatedPose.getRotation().getRadians()
        };
        SmartDashboard.putNumberArray("pose", poseArray);
    }


    @Override
    public void autonomousPeriodic() {

    }

    @Override
    public void teleopPeriodic() {

    }

    @Override
    public void autonomousInit() {

    }

    @Override
    public void teleopInit() {

    }

    @Override
    public void disabledInit() {

    }

    @Override
    public void disabledPeriodic() {

    }

}
