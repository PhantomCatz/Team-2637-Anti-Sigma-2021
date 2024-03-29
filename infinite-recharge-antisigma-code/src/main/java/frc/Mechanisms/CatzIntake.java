package frc.Mechanisms;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.fasterxml.jackson.databind.jsontype.PolymorphicTypeValidator.Validity;
import com.revrobotics.CANDigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class CatzIntake {
    private final int INTAKE_FIGURE_8_MC_CAN_ID = 10;
    private final int INTAKE_ROLLER_MC_CAN_ID = 11;
    private final int INTAKE_DEPLOY_DS_A_ID = 5;
    private final int INTAKE_DEPLOY_DS_B_ID = 6;

    private final double MTR_POWER_FIGURE8 = -0.9;
    private final double MTR_POWER_ROLLER = 0.6;

    // private final double COMPRESSION_POWER = 0.3;

    // private final double INTAKE_MOTOR_POWER_START_DEPLOY = 0.25;
    // private final double INTAKE_MOTOR_POWER_END_DEPLOY = 0.0;
    // private final double INTAKE_MOTOR_POWER_START_STOW = -0.25;
    // private final double INTAKE_MOTOR_POWER_END_STOW = -0.2;

    final double INTAKE_THREAD_WAITING_TIME = 0.050;
    final double DEPLOY_REDUCE_POWER_TIME_OUT_SEC = 0.400;
    final double STOW_REDUCE_POWER_TIME_OUT_SEC = 0.350;


    private WPI_VictorSPX intakeFigure8MtrCtrl;
    public WPI_TalonSRX intakeRollerMtrCtrl; // changed to public because dataport is being used on drivetrain?

    private DoubleSolenoid intakeDeploySolenoid;

    private CANDigitalInput intakeDeployedLimitSwitch;
    private CANDigitalInput intakeStowedLimitSwitch;

    private int deployPowerCountLimit = 0;
    private int stowPowerCountLimit = 0;
    private int timeCounter = 0;

    private Thread intakeThread;

    boolean intakeDeployed = false;

    public CatzIntake() {
        intakeFigure8MtrCtrl = new WPI_VictorSPX(INTAKE_FIGURE_8_MC_CAN_ID);
        intakeRollerMtrCtrl = new WPI_TalonSRX(INTAKE_ROLLER_MC_CAN_ID);
        intakeDeploySolenoid = new DoubleSolenoid(INTAKE_DEPLOY_DS_A_ID, INTAKE_DEPLOY_DS_B_ID);

        // intakeDeploySolenoid = new CANSparkMax (INTAKE_DEPLOY_MC_CAN_ID,
        // MotorType.kBrushless);

        // intakeDeployedLimitSwitch =
        // intakeDeploySolenoid.getForwardLimitSwitch(LimitSwitchPolarity.kNormallyOpen);
        // intakeStowedLimitSwitch =
        // intakeDeploySolenoid.getReverseLimitSwitch(LimitSwitchPolarity.kNormallyOpen);

        // Reset configuration
        intakeFigure8MtrCtrl.configFactoryDefault();
        intakeRollerMtrCtrl.configFactoryDefault();

        // intakeDeploySolenoid.restoreFactoryDefaults();

        // Set roller MC to coast intakeMode
        intakeFigure8MtrCtrl.setNeutralMode(NeutralMode.Coast);
        intakeRollerMtrCtrl.setNeutralMode(NeutralMode.Coast);

        // Set deploy MC to brake intakeMode
        // intakeDeploySolenoid.setIdleMode(IdleMode.kBrake);

        deployPowerCountLimit = (int) Math.round((DEPLOY_REDUCE_POWER_TIME_OUT_SEC / INTAKE_THREAD_WAITING_TIME) + 0.5);
        stowPowerCountLimit = (int) Math.round((STOW_REDUCE_POWER_TIME_OUT_SEC / INTAKE_THREAD_WAITING_TIME) + 0.5);
        SmartDashboard.putNumber("stow count limit", stowPowerCountLimit);
        SmartDashboard.putNumber("deploy count limit", deployPowerCountLimit);
    }

    // ---------------------------------------------ROLLER---------------------------------------------
    public void intakeRollerIn() {
        intakeFigure8MtrCtrl.set(ControlMode.PercentOutput, -MTR_POWER_FIGURE8);
        intakeRollerMtrCtrl.set(ControlMode.PercentOutput, -MTR_POWER_ROLLER);
    }

    public void intakeRollerOut() {
        intakeFigure8MtrCtrl.set(ControlMode.PercentOutput, MTR_POWER_FIGURE8);
        intakeRollerMtrCtrl.set(ControlMode.PercentOutput, MTR_POWER_ROLLER);
    }

    public void intakeRollerOff() {
        intakeFigure8MtrCtrl.set(ControlMode.PercentOutput, 0.0);
        intakeRollerMtrCtrl.set(ControlMode.PercentOutput, 0.0);
    }

    // ---------------------------------------------DEPLOY/STOW---------------------------------------------


    public void deployIntake() 
    {
        intakeDeploySolenoid.set(Value.kForward);
        //Timer.delay(1);
        //intakeDeploySolenoid.set(Value.kOff);

    }

    public void stowIntake() 
    {
        intakeDeploySolenoid.set(Value.kReverse);
        //Timer.delay(1);
        //intakeDeploySolenoid.set(Value.kOff);
    }

    public void applyBallCompression() {
        intakeDeploySolenoid.set(DoubleSolenoid.Value.kForward);
        /*
         * if(intakeDeployed == true) { intakeDeploySolenoid.set(COMPRESSION_POWER); }
         */
    }

    public Value getDeployMotorPower()
    {
        return intakeDeploySolenoid.get();
    }

    // ---------------------------------------------Intake Limit Switches---------------------------------------------   
    /*
    public boolean getDeployedLimitSwitchState()
    {
        return intakeDeployedLimitSwitch.get();
    }
    public boolean getStowedLimitSwitchState()
    {
        return intakeStowedLimitSwitch.get();
    }*/

    //---------------------------------------------Autonomous pickup---------------------------------------------------

    public void intakePowerCell()
    {
        Robot.auton.driveStraightIntake(14, Robot.auton.MIN_FPS_VELOCITY, 500);
        if(Robot.indexer.ballPresent())
        {
            intakeRollerOff();
        }
        
    }
}