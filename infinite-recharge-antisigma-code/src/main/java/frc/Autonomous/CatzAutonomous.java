package frc.Autonomous;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.DataLogger.CatzLog;
import frc.DataLogger.DataCollection;
import frc.Mechanisms.*;
import frc.robot.*;
import com.kauailabs.navx.frc.AHRS;

public class CatzAutonomous
{

    public static boolean checkBoxL;
    public static boolean checkBoxM;
    public static boolean checkBoxR;

    public boolean prev_boxL = false;
	public boolean prev_boxM = false;
	public boolean prev_boxR = false;

    public final double STOP_THRESHOLD_ANGLE = 2;
    public final double SLOW_THRESHOLD_ANGLE = 20;

    public boolean runningRadialTurn = false;
    public double angleGoal;
    public double angleToTurn;

    public double currentEncCountRt;
    public double currentEncCountLt;
    public double turnRateRadians;

    public double r1;
    public double r2;
    public double s1Dot;
    public double s2Dot;
    public double s1Conv;
    public double s2Conv;

    public Timer turnT;
    public Timer driveTWait;
    public Timer shooterTimer;
    public Timer driveStraightTimer;

    public final double WAIT_TIME_THRESHOLD = 5.0;
    public double totalTime = 0;

    public double lVelocity;
    public double rVelocity;

    public int leftInitialEncoderCnt;
    public int rightInitialEncoderCnt;

    public boolean runningDistanceDrive = false;
    public boolean driveBackwards = false;
    public boolean monitoringTime = false;

    public double distanceGoal;
    public double distanceMoved;
    public final double STOP_THRESHOLD_DIST = 1;
    public final double SLOW_THRESHOLD_DIST = 51; //make larger(orig 30in)

    public final double ENCODER_COUNTS_PER_INCH_LT = 1014.5; //without weight: 1046.6
    public final double ENCODER_COUNTS_PER_INCH_RT = 964;    //without weight: 1025.7

    public final double TO_RADIANS = Math.PI/180;
    public final double TO_DEGREES = 180/Math.PI;

    public final int DRIVE_STRAIGHT_TIMEOUT = 30;
    public boolean inDriveStraight = false;


    public final double MIN_VELOCITY_LIMIT_FPS = 5.0;
    public final double CONVERTED_MIN_VELOCITY_LIMIT = convertVelocity(MIN_VELOCITY_LIMIT_FPS);// 3000;
    public final double VELOCITY_DECREASE_RATE = 1.0;//0.77; // make this smallerorig 0.92


    public double targetVelocity = 0.0;

    

    public final double FPS_TO_CNTS_PER_100MS = (12.0/1.0) * 1.0/Robot.driveTrain.encCountsToInches*(1.0/10.0);

    /*******************************************************************************
     * Drive straight Start and decr constants
     * 
    */

    double maxSpeedFPS = 0.0;

    public final double MIN_FPS_VELOCITY = 5.0;

    public final double VELOCITY_DECREASE_RATE_8 = 0.955;
    public final double VELOCITY_DECREASE_RATE_10 = 0.93;
    public final double VELOCITY_DECREASE_RATE_12 = 0.915;
    public final double VELOCITY_DECREASE_RATE_13 = 0.91;
    public final double VELOCITY_DECREASE_RATE_14 = 0.9;
    

    public double velDecrRate = 1.0;


    public double decelDist;

    public final double MIN_DIST_TO_TARGET_8 = 25.2 + 10.0;
    public final double MIN_DIST_TO_TARGET_10 = 28.8 + 10.0;
    public final double MIN_DIST_TO_TARGET_12 = 32.4 + 10.0;
    public final double MIN_DIST_TO_TARGET_13 = 34.2 + 10.0;
    public final double MIN_DIST_TO_TARGET_14 = 36.0 + 10.0;
    	/***************************************************************************
	 * PID Turn Constants
	 ***************************************************************************/
    static public double PID_TURN_THRESHOLD   = 0.5;

	/***************************************************************************
	 * PID_TURN_DELTAERROR_THRESHOLD_HI - Delta Error Values larger than this are
	 * considered invalid and will be ignored PID_TURN_DELTAERROR_THRESHOLD_LO -
	 * When drivetrain power drops below the PID_TURN_MIN_xxx_POWER, we will check
	 * to see if deltaError is below this threshold before setting power at
	 * PID_TURN_MIN_xxx_POWER.
	 ***************************************************************************/
	final static public double PID_TURN_DELTAERROR_THRESHOLD_HI = 4.0;
	final public static double PID_TURN_DELTAERROR_THRESHOLD_LO = 0.11;

	final static public double PID_TURN_FILTER_CONSTANT    = 0.7;
	      static public double PID_TURN_POWER_SCALE_FACTOR = 1.0;

	      static public double PID_TURN_KP = 0.08;
	      static public double PID_TURN_KI = 0.0;
	      static public double PID_TURN_KD = 0.012; // 0.0744

	final static public double PID_TURN_INTEGRAL_MAX =  1.0;
	final static public double PID_TURN_INTEGRAL_MIN = -1.0;

	final public static double PID_TURN_MIN_POS_POWER = 0.6; // 0.4 is min power to move robot when it is stopped
	final public static double PID_TURN_MIN_NEG_POWER = -PID_TURN_MIN_POS_POWER;

	
	/***************************************************************************
	 * PID Turn Variables
	 ***************************************************************************/
	static Timer functionTimer;
    static Timer pdTimer;
    
    final public static double DRIVE_MAX_POS_POWER =  1.0;
	final public static double DRIVE_MAX_NEG_POWER = -1.0;

    final static public double NAVX_RESET_WAIT_TIME = 0.2;

    static double pidTurnkP = PID_TURN_KP;
    static double pidTurnkI = PID_TURN_KI;
    static double pidTurnkD = PID_TURN_KD;

	static double currentError; 
	static double deltaError;
	static double derivative;
	static double deltaT;

	static double power;

	static double previousError;
	static double totalError;

	static double currentAngle;
	static double currentAngleAbs;
	static double targetAngle;
	static double targetAngleAbs;

	static double timeout;
	static double loopDelay = 0.015;

	static double previousDerivative = 0;

	static boolean tuningMode = false;
	static boolean debugMode = false;



    public CatzAutonomous()
    {
        turnT        = new Timer();
        driveTWait   = new Timer();
        shooterTimer = new Timer();
        driveStraightTimer = new Timer();
    }

    public void resetTotalTime()
    {
        totalTime = 0;
    }

    public boolean simulateShoot()
    {
        boolean status = false;

        if(shooterTimer.get() > 3)
        {
            status = true;
        }
        return status;
    }

    

   


    /*******8*****88********8*******************88*****8*8**8****8***8*8*****
    * 
    * Convert velocity from ft/sec to counts/100msec
    *
    ************************************************************************/
    public double convertVelocity(double ftPerSec) 
    {
        double cntsPer100Ms = ftPerSec * FPS_TO_CNTS_PER_100MS; 
        return cntsPer100Ms;
    }

    public void setDriveParam(double distance)
    {
        distanceGoal = distance;// - 8;//where intake roller should be
        if(distanceGoal > 0 && distanceGoal <= 70)
        {
            velDecrRate = VELOCITY_DECREASE_RATE_8;
            decelDist = MIN_DIST_TO_TARGET_8;
            maxSpeedFPS = 8.0;
        }
        else if(distanceGoal > 70 && distanceGoal <= 95)
        {
            velDecrRate = VELOCITY_DECREASE_RATE_10;
            decelDist = MIN_DIST_TO_TARGET_10;
            maxSpeedFPS = 10.0;
        }
        else if(distanceGoal > 95 && distanceGoal <= 120)
        {
            velDecrRate = VELOCITY_DECREASE_RATE_12;
            decelDist = MIN_DIST_TO_TARGET_12;
            maxSpeedFPS = 12.0;
        }
        else if(distanceGoal > 120 && distanceGoal <= 150)
        {
            velDecrRate = VELOCITY_DECREASE_RATE_13;
            decelDist = MIN_DIST_TO_TARGET_13;
            maxSpeedFPS = 13.0;
        }
        else if(distanceGoal > 150 && distanceGoal <= 180)
        {
            velDecrRate = VELOCITY_DECREASE_RATE_14;
            decelDist = MIN_DIST_TO_TARGET_14;
            maxSpeedFPS = 14.0;
        }
    }



    //-------------------------------------PATH A----------------------------------------------
    public void drivePathAYellow()
    {

    }

    public void drivePathAPurple()
    {

    }

    //-------------------------------------PATH B----------------------------------------------
    public void drivePathBYellow() //check colors
    {

    }

    public void drivePathBPurple()
    {
        
    }


    /*
     *Distance: input distance in inches that the robot has to go
     *maxSpeedFPS: input speed in ft/sec
     *timeout:  input timeout time in 
     */
    
    public void driveStraight(double distance, double timeout)
    {
        Robot.driveTrain.shiftToHighGear();   
        setDriveParam(distance);
        System.out.println("in drv straight");
        
        rightInitialEncoderCnt = Robot.driveTrain.drvTrainMtrCtrlRTFrnt.getSelectedSensorPosition(0);

        
        distanceGoal = Math.abs(distanceGoal); 

        if (distance < 0)
        {
            targetVelocity = -convertVelocity(maxSpeedFPS);//cnts/100ms
        }
        else    
        {
            targetVelocity = convertVelocity(maxSpeedFPS);
        }

        //driveStraightLogConfig();
        
        Robot.driveTrain.setTargetVelocity(targetVelocity);

        monitorEncoderPosition(rightInitialEncoderCnt, timeout);
    }
    

    public boolean monitorEncoderPosition(int initialEncoderCount, double timeoutTime)
    {
        Robot.navx.reset();
        CatzLog data;
        driveStraightTimer.reset();
        driveStraightTimer.start();

        boolean completed = false;
        boolean done      = false;
        boolean intakeRunning = false;
        double distanceRemaining = 0.0;
        double currentVelocityRt  = 0.0;
        double currentVelocityLt  = 0.0;
        double deltaCounts;
        double currentTime = 0.0;
        
        Robot.dataCollection.isFirst = true;

       
       while(done == false)
       {
           System.out.println("monitor enc pos");
           System.out.println("NavX:   " + Robot.navx.getAngle());
            currentEncCountRt = Robot.driveTrain.drvTrainMtrCtrlRTFrnt.getSelectedSensorPosition(0);//for left and right
            currentEncCountLt = Robot.driveTrain.drvTrainMtrCtrlLTFrnt.getSelectedSensorPosition(0);
            currentVelocityLt = Robot.driveTrain.getIntegratedEncVelocity("LT");
            currentVelocityRt = Robot.driveTrain.getIntegratedEncVelocity("RT"); //for left and right
           

            deltaCounts = currentEncCountRt - initialEncoderCount;
            distanceMoved = Math.abs((double) (deltaCounts * Robot.driveTrain.encCountsToInches) );
            
            distanceRemaining = distanceGoal - distanceMoved; //distance in inches (error)

            currentTime = driveStraightTimer.get();
            
            if (distanceRemaining < STOP_THRESHOLD_DIST)
            {
                targetVelocity = 0.0; //CONVERTED_MIN_VELOCITY_LIMIT;
                Robot.driveTrain.setTargetVelocity(targetVelocity);
                completed = true;
                done = true;
                
            }
            else 
            {
                if (distanceRemaining <= decelDist)
                {


                    if(targetVelocity > CONVERTED_MIN_VELOCITY_LIMIT )
                    {   if(intakeRunning == false)
                        {
                            Robot.intake.intakeRollerIn();
                            intakeRunning = true;
                        }
                        
                        targetVelocity = targetVelocity * velDecrRate;
                        Robot.driveTrain.setTargetVelocity(targetVelocity);
                    }    
                    
                }
                
                if(currentTime > timeoutTime)
                {
                    targetVelocity = 0.0;
                    Robot.driveTrain.setTargetVelocity(targetVelocity);
                    completed = false;
                    done = true;
                }
            }
            if (Robot.dataCollection.isFirst == true)
            {
                data = new CatzLog(distanceGoal,maxSpeedFPS, decelDist,velDecrRate, CONVERTED_MIN_VELOCITY_LIMIT,
                            CONVERTED_MIN_VELOCITY_LIMIT, 
                            Robot.driveTrain.LT_PID_P,Robot.driveTrain.LT_PID_I,
                            Robot.driveTrain.LT_PID_D,Robot.driveTrain.LT_PID_F,Robot.driveTrain.encCountsToInches,
                            Robot.driveTrain.currentDrvTrainGear,
                            Robot.driveTrain.DRIVE_STRAIGHT_PID_TUNING_CONSTANT, Robot.pdp.getVoltage(),
                            Robot.driveTrain.RT_PID_P,Robot.driveTrain.RT_PID_F);
                Robot.dataCollection.isFirst = false;
            }
            else 
            {
                data = new CatzLog(currentTime, targetVelocity, currentVelocityRt,
                                                                Robot.driveTrain.drvTrainMtrCtrlRTFrnt.getClosedLoopError(0), 
                                                                currentEncCountRt, currentVelocityLt, Robot.driveTrain.drvTrainMtrCtrlLTFrnt.getClosedLoopError(0), 
                                                                currentEncCountLt, distanceRemaining, Robot.navx.getAngle(),
                                                                -999.0, -999.0, -999.0, -999.0,-999.0, -999.0);
                
            }
            Robot.dataCollection.logData.add(data);
            Robot.dataCollectionTimer.delay(0.03);
               
 
       }
       Robot.intake.intakeRollerOff();
        /*while( Math.abs(currentVelocityRt) > 50  && Math.abs(currentVelocityLt) > 50)
        {

            currentEncCountRt = Robot.driveTrain.drvTrainMtrCtrlRTFrnt.getSelectedSensorPosition(0);//for left and right
            currentEncCountLt = Robot.driveTrain.drvTrainMtrCtrlLTFrnt.getSelectedSensorPosition(0);
            currentVelocityRt = Robot.driveTrain.getIntegratedEncVelocity("RT"); //for left and right
            currentVelocityLt = Robot.driveTrain.getIntegratedEncVelocity("LT");

            currentTime = driveStraightTimer.get();

            deltaCounts = currentEncCountRt - initialEncoderCount;
            distanceMoved = Math.abs((double) (deltaCounts * Robot.driveTrain.encCountsToInches) );
            
            distanceRemaining = distanceGoal - distanceMoved; //distance in inches (error)

            data = new CatzLog(currentTime, targetVelocity, currentVelocityRt,
                                                                Robot.driveTrain.drvTrainMtrCtrlRTFrnt.getClosedLoopError(0), 
                                                                currentEncCountRt, currentVelocityLt, Robot.driveTrain.drvTrainMtrCtrlLTFrnt.getClosedLoopError(0), 
                                                                currentEncCountLt, distanceRemaining, Robot.navx.getAngle(),
                                                                -999.0, -999.0, -999.0, -999.0,-999.0, -999.0);
                
            
            Robot.dataCollection.logData.add(data);
            Robot.dataCollectionTimer.delay(0.03);
        }  */    

        return completed;

    
    }

    public void driveStraightIntake(double distance, double maxSpeedFPS, double timeout)
    {
        System.out.println("in drv straight");
        
        rightInitialEncoderCnt = Robot.driveTrain.drvTrainMtrCtrlRTFrnt.getSelectedSensorPosition(0);
        
        

        distanceGoal = Math.abs(distance); 

        if (distance < 0)
        {
            targetVelocity = -convertVelocity(maxSpeedFPS);//cnts/100ms
        }
        else    
        {
            targetVelocity = convertVelocity(maxSpeedFPS);
        }

        //driveStraightLogConfig();
        
        Robot.driveTrain.setTargetVelocity(targetVelocity);

        monitorEncoderPositionIntake(rightInitialEncoderCnt, timeout);
    }

    public boolean monitorEncoderPositionIntake(int initialEncoderCount, double timeoutTime)
    {
        CatzLog data;
        driveStraightTimer.reset();
        driveStraightTimer.start();

        boolean completed = false;
        boolean done      = false;
        double distanceRemaining = 0.0;
        double currentVelocityRt  = 0.0;
        double currentVelocityLt  = 0.0;
        double deltaCounts;
        double currentTime = 0.0;
        double halfWay = distanceGoal * 0.5;
        
        Robot.dataCollection.isFirst = true;

        
        
       while(done == false)
       {
           System.out.println("monitor enc pos");
           System.out.println("NavX:   " + Robot.navx.getAngle());
            currentEncCountRt = Robot.driveTrain.drvTrainMtrCtrlRTFrnt.getSelectedSensorPosition(0);//for left and right
            currentEncCountLt = Robot.driveTrain.drvTrainMtrCtrlLTFrnt.getSelectedSensorPosition(0);
            currentVelocityRt = Robot.driveTrain.getIntegratedEncVelocity("RT"); //for left and right
            currentVelocityLt = Robot.driveTrain.getIntegratedEncVelocity("LT");

            deltaCounts = currentEncCountRt - initialEncoderCount;
            distanceMoved = Math.abs((double) (deltaCounts * Robot.driveTrain.encCountsToInches) );
            
            distanceRemaining = distanceGoal - distanceMoved; //distance in inches (error)

            currentTime = driveStraightTimer.get();
            
            if (distanceRemaining < STOP_THRESHOLD_DIST)
            {
                targetVelocity = 0.0;
                Robot.driveTrain.setTargetVelocity(targetVelocity);
                completed = true;
                done = true;
                
            }
            else 
            {
                if(currentTime > timeoutTime)
                {
                    targetVelocity = 0.0;
                    Robot.driveTrain.setTargetVelocity(targetVelocity);
                    completed = false;
                    done = true;
                }
            }
            
            data = new CatzLog(currentTime, targetVelocity, currentVelocityRt,
                                                            Robot.driveTrain.drvTrainMtrCtrlRTFrnt.getClosedLoopError(0), 
                                                            currentEncCountRt, currentVelocityLt, Robot.driveTrain.drvTrainMtrCtrlLTFrnt.getClosedLoopError(0), 
                                                            currentEncCountLt, distanceRemaining, Robot.navx.getAngle(),
                                                            -999.0, -999.0, -999.0, -999.0,-999.0, -999.0);
            
            Robot.dataCollection.logData.add(data);
            Robot.dataCollectionTimer.delay(0.03);

       }
       return completed;
    }   


    public void PIDturn(double degreesToTurn, double timeoutSeconds, double maxpower) {
		Robot.navx.reset();
		Timer.delay(NAVX_RESET_WAIT_TIME);

		pdTimer       = new Timer();
		functionTimer = new Timer();
		functionTimer.reset();
		functionTimer.start(); 

		boolean done      = false;
		boolean firstTime = true;

		setPidValues(degreesToTurn);

		previousError = 0.0;
		totalError    = 0.0;

		currentAngle  = Robot.navx.getAngle();
		targetAngle   = degreesToTurn + currentAngle;
		currentError  = targetAngle   - currentAngle;

		targetAngleAbs = Math.abs(targetAngle);


        logPIDValues();
		

		pdTimer.reset();
		pdTimer.start();
		while (done == false) {
			currentAngle = Robot.navx.getAngle();

			deltaT = pdTimer.get();
			pdTimer.stop();
			pdTimer.reset();
			pdTimer.start();

			currentAngleAbs = Math.abs(currentAngle);


			// calculates proportional term
			currentError = targetAngle - currentAngle;

			if (Math.abs(currentError) < PID_TURN_THRESHOLD) {
				done = true;
			} else {
				if (functionTimer.get() > timeoutSeconds) {
					done = true;
				} else {
					/************************************************************
					 * Calculate derivative term If this is the first time through the loop, we
					 * don't have a previousError or previouisDerivative value, so we will just set
					 * derivative to zero.
					 ************************************************************/
					deltaError = currentError - previousError;

					if (firstTime == false) {

						/*************************************************************
						 * Filter out invalid values (noise) as we don't want the control loop to react
						 * to these. Invalid values can occur due to mechanical imperfections causing
						 * the drivetrain to bind/release as it is turning, missed samples, etc. When
						 * the control loop reacts to these unexpected jumps, it will lead to large
						 * swings in power as it tries to correct for a large intermittent error that
						 * comes & goes. This may be seen as the robot shaking during the turn.
						 *
						 * An invalid value is characterized as one o - jumping to zero when we are not
						 * close to targetAngle - Change in delta error has exceeded a threshold
						 *
						 * If we have an invalid value, use the previous derivative value.
						 *************************************************************/
						if ((deltaError == 0.0) && (Math.abs(currentError) > 3.0)) {
							derivative = previousDerivative;
						} else {

							if (Math.abs(deltaError) > PID_TURN_DELTAERROR_THRESHOLD_HI) {
								derivative = previousDerivative;

							} else {
								/**********************************************************
								 * We have a good deltaError value. Filter the derivative value to smooth out
								 * jumps in derivative value
								 **********************************************************/
								derivative = PID_TURN_FILTER_CONSTANT * previousDerivative
										+ ((1 - PID_TURN_FILTER_CONSTANT) * (deltaError / deltaT));
							}
						}
					} else {
						firstTime = false;
						derivative = 0;
					}

					// Save values for next iteration
					previousDerivative = derivative;
					previousError      = currentError;

					/*******************************************************************
					 * Calculate integral term
					 *
					 * Check if we are entering saturation. If we are cap totalError at max value
					 * (make sure the integral term doesn't get too big or small)
					totalError += currentError * deltaT;
					if (totalError >= PID_TURN_INTEGRAL_MAX)
						totalError = PID_TURN_INTEGRAL_MAX;
					if (totalError <= PID_TURN_INTEGRAL_MIN)
						totalError = PID_TURN_INTEGRAL_MIN;
					 *******************************************************************/

					/*******************************************************************
					 * Calculates drivetrain power
					 *******************************************************************/
                    power = PID_TURN_POWER_SCALE_FACTOR
                            * ((pidTurnkP * currentError) + (PID_TURN_KI * totalError) + (PID_TURN_KD * derivative));

					// Verify we have not exceeded max power when turning right or left
					if (power > DRIVE_MAX_POS_POWER)
						power = DRIVE_MAX_POS_POWER;

					if (power < DRIVE_MAX_NEG_POWER)
						power = DRIVE_MAX_NEG_POWER;

					/**********************************************************************
					 * We need to make sure drivetrain power doesn't get too low but we also need to
					 * allow the robot to gradually brake. The brake condition is defined as when
					 * deltaError is > PID_TURN_DELTAERROR_THRESHOLD_LO If deltaError is <
					 * PID_TURN_DELTAERROR_THRESHOLD_LO, then we will set power to
					 * PID_TURN_MIN_xxx_POWER.
					 **********************************************************************/
					if (power >= 0.0) {
						if (power < PID_TURN_MIN_POS_POWER && Math.abs(deltaError) < PID_TURN_DELTAERROR_THRESHOLD_LO)
							power = PID_TURN_MIN_POS_POWER;
					} else if (power < 0.0) {
						if (power > PID_TURN_MIN_NEG_POWER && Math.abs(deltaError) < PID_TURN_DELTAERROR_THRESHOLD_LO)
							power = PID_TURN_MIN_NEG_POWER;
					}

					/*******************************************************************
					 * Cmd robot to turn at new power level Note: Power will be positive if turning
					 * right and negative if turning left
					 *******************************************************************/
                    Robot.driveTrain.setTargetPower(Math.min(power,maxpower));
                    logDebugData();
					printDebugData();
					Timer.delay(loopDelay);
				}
            }
            
		}

		/**********************************************************************
		 * We're at targetAngle or timed out. Stop the robot and do final cleanup. -
		 * Print out last set of debug data (note that this may not be a complete set of
		 * data) - Stop timers
		 **********************************************************************/
	currentAngle = Robot.navx.getAngle();
	printDebugData();
		Robot.driveTrain.setTargetPower(0.0); // makes robot stop
	currentAngle = Robot.navx.getAngle();
	printDebugData();

		functionTimer.stop();
		pdTimer.stop();
	}



    /***************************************************************************
    *
    * setPidValues()
    * 
    ***************************************************************************/
    public static void setPidValues(double degreesToTurn) {
        double degreesToTurnAbs;

        degreesToTurnAbs = Math.abs(degreesToTurn);

        if (degreesToTurnAbs <= 25.0) {
            pidTurnkP = 0.090;
            pidTurnkD = 0.024;
        }
        if (degreesToTurnAbs <= 30.0) {
            pidTurnkP = 0.110;   //0.126 at 12.0 V;
            pidTurnkD = 0.026;
            PID_TURN_THRESHOLD = 0.75;
            loopDelay = 0.007;
        }
        else if (degreesToTurnAbs <= 35.0) {
            pidTurnkP = 0.090;
            pidTurnkD = 0.020;
        }
        else if (degreesToTurnAbs <= 40.0) {
            pidTurnkP = 0.086;
            pidTurnkD = 0.024;
        }
        else if (degreesToTurnAbs <= 45.0) {
            pidTurnkP = 0.090;
            pidTurnkD = 0.030;
            PID_TURN_THRESHOLD = 0.75;
            loopDelay = 0.007;
        }
        else if (degreesToTurnAbs <= 50.0) {
            pidTurnkP = 0.100;
            pidTurnkD = 0.028;
            PID_TURN_THRESHOLD = 0.75;
            loopDelay = 0.007;
        }
        else { //if degreesToTurnAbs > 50.0
            pidTurnkP = 0.080;  //PID_TURN_KP;
            pidTurnkD = 0.030;  //PID_TURN_KD;
            PID_TURN_THRESHOLD = 0.5;
            loopDelay = 0.010;
        }

    }   //End of setPidValues()



    /***************************************************************************
    *
    * setPIDTurnDebugModeEnabled()
    * 
    ***************************************************************************/
	public static void setPIDTurnDebugModeEnabled(boolean enabled) {
		debugMode = enabled;
	}

	public static boolean isTuningModeEnabled() {
		return tuningMode;
	}

    /***************************************************************************
    *
    * printDebugInit()
    * 
    ***************************************************************************/
	public static void printDebugInit() {
		if (debugMode == true) {
            System.out.println("navX: *** PID Turn **********************************************************");
            System.out.printf("navX: kP, %.3f, kI, %.3f, kD, %.3f\n", pidTurnkP, pidTurnkI, pidTurnkD);
            System.out.printf("navX: currentAngle,   %.3f, targetAngle, %.3f\n", currentAngle, targetAngle);
            System.out.printf("navX: ErrThreshold,   %.3f\n", PID_TURN_THRESHOLD);
            System.out.printf("navX: DerivFiltConst, %.3f\n", PID_TURN_FILTER_CONSTANT);
            System.out.printf("navX: MinPosPwr,      %.3f\n", PID_TURN_MIN_POS_POWER);
            System.out.printf("navX: MinNegPwr,      %.3f\n", PID_TURN_MIN_NEG_POWER);
            System.out.printf("navX: delErrThreshHi, %.3f\n", PID_TURN_DELTAERROR_THRESHOLD_HI);
            System.out.printf("navX: delErrThreshLo, %.3f\n", PID_TURN_DELTAERROR_THRESHOLD_LO);
		}
	}


	/***************************************************************************
    *
    * printDebugHeader()
    * 
    ***************************************************************************/
    public static void printDebugHeader() {
        if (debugMode == true) {
            System.out.print("navX: time,deltaT,currAngle,currError,deltaError,deriv,power\n");
        }
    }


    /***************************************************************************
    *
    * printDebugData()
    * 
    ***************************************************************************/
    public static void printDebugData() {
        if (debugMode == true) {

            System.out.printf("navX: %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f\n", functionTimer.get(), deltaT,
                    currentAngle, currentError, deltaError, derivative, power);
        }
    }
    
    /***************************************************************************
    *
    * Access Methods
    * 
    ***************************************************************************/
    public static double getCurrTime()
    {
        return functionTimer.get();
    }

    public static double getDeltaT()
    {
        return pdTimer.get();
    }

    public static double getCurrAngle()
    {
        return Robot.navx.getAngle();
    }

    public static  double getCurrError()
    {
        double CurrError = targetAngle - currentAngle;
        return CurrError;
    }
 
    public static double getDeltaError()
    {
        double dError = currentError - previousError;
        return dError;
    }

    public static double getCurrPower()
    {
        return power;
    }

    public static void logDebugData()
    {
        CatzLog data;
        data = new CatzLog(functionTimer.get(), deltaT, 
        currentAngle, currentError, deltaError, derivative, power, -999.0, -999.0, -999.0, -999.0,
        -999.0, -999.0, -999.0, -999.0, -999.0);
        Robot.dataCollection.logData.add(data);
    }

    public void logPIDValues()
    {
        CatzLog data; 
        data = new CatzLog(pidTurnkP, pidTurnkD, loopDelay, PID_TURN_FILTER_CONSTANT,-999.0, -999.0, -999.0, -999.0,
        -999.0, -999.0, -999.0, -999.0, -999.0,-999.0, -999.0,-999.0);
        Robot.dataCollection.logData.add(data);
    }

    /*public void driveStraightLogConfig()
    {
        System.out.println("T0: " + Robot.driveTrain.PID_P + "," 
                                  + Robot.driveTrain.PID_I + "," 
                                  + Robot.driveTrain.PID_D + "," 
                                  + Robot.driveTrain.PID_F + ","  
                                  + Robot.driveTrain.encCountsToInches + "," 
                                  + Robot.driveTrain.currentDrvTrainGear + "," 
                                  + distanceGoal + ","
                                  + STOP_THRESHOLD_DIST
        );
        
    }*/
}