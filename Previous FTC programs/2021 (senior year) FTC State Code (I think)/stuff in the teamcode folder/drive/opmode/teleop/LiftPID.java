package org.firstinspires.ftc.teamcode.drive.opmode.teleop;

public class LiftPID {
    double previouserror = 0;
    double error = 0;
    double Interval = 0.02;
    double derivative = 0;
    double integralSum = 0;

    double KP = 1;
    double KI = 0;
    double KD = 0;
    double KG = 0;
    double IntRange = 0;
    double output = 0;
    public LiftPID(double kP,double kI,double kD, double kG, double intRange)
    {
        KP = kP;
        KI = kI;
        KD = kD;
        KG = kG;
        IntRange = intRange;
    }
    public double Calculate(double CurrentPos, double TargetPos,double interval)
    {
    Interval= interval;
    // calculate the error
     error = TargetPos - CurrentPos;
     // rate of change of the error
        derivative = (error - previouserror) / interval;
        if(error<IntRange)
        integralSum = integralSum + (error * interval);
        output = (KP * error) + (KI * integralSum) + (KD * derivative) + KG;
        previouserror = error;
        return output;
    }

    public void reset()
    {
        previouserror = 0;
        error = 0;
        Interval = 0.02;
        derivative = 0;
        integralSum = 0;
    }
}
