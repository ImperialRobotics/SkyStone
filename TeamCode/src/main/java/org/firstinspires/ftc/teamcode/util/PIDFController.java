package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.util.MathUtil;

public class PIDFController {

    private double error, lastError, errorSum;
    private double setpoint, tolerance;

    private boolean inputBounded, outputBounded;
    private double minInput, maxInput, minOutput, maxOutput, maxSum;

    private PIDFCoefficients pidf;

    private double lastTime;

    //----------------------------------------------------------------------------------------------
    // Constructors
    //----------------------------------------------------------------------------------------------

    public PIDFController(PIDFCoefficients pidf,
                          double minInput, double maxInput, double minOutput, double maxOutput,
                          double maxSum) {
        this.pidf = pidf;
        inputBounded = true;
        outputBounded = true;
        this.minInput = minInput;
        this.maxInput = maxInput;
        this.minOutput = minOutput;
        this.maxOutput = maxOutput;
        this.maxSum = maxSum;
        lastTime = Double.NaN;
    }


    public PIDFController(PIDFCoefficients pidf) {
        this(pidf, -180, 180, -1.0, 1.0, Double.POSITIVE_INFINITY);
    }

    //----------------------------------------------------------------------------------------------
    // Getters and Setters
    //----------------------------------------------------------------------------------------------

    public PIDFCoefficients getPIDFCoefficients() { return pidf; }

    /**
     * Sets the percentage error which is considered tolerable for use with the onTarget method
     * @param tolerance decimal error which is tolerable (input of 0.15 = 15 percent tolerance)
     */
    public void setTolerance(double tolerance) {
        this.tolerance = tolerance;
    }

    /**
     * Sets the maximum value of the sum of the error in the calculate method so that the integral
     * term does not windup and overwhelm the output given;
     * @param maxIntegral maximum value of the integral term in the calculate method
     */
    public void setMaxSum(double maxIntegral) {
        this.maxSum = maxIntegral;
    }

    public void setInputRange(double minInput, double maxInput) {
        this.minInput = minInput;
        this.maxInput = maxInput;
        setSetpoint(setpoint);
    }

    public void setOutputRange(double minOutput, double maxOutput) {
        this.minOutput = minOutput;
        this.maxOutput = maxOutput;
        setSetpoint(setpoint);
    }

    /**
     * sets the set point (target position) of the PIDFController, clipping it to between minInput
     * and maxInput
     * @param setpoint the set point of the PIDFController
     */
    public void setSetpoint(double setpoint) {
        if (maxInput > minInput) {
            this.setpoint = Range.clip(setpoint, minInput, maxInput);
        } else {
            this.setpoint = setpoint;
        }
    }

    //----------------------------------------------------------------------------------------------
    // Methods
    //----------------------------------------------------------------------------------------------

    /**
     * Calculates error between setpoint and input of the system, wrapping error if input bounds
     * have been set
     * @param input current position of the system
     */
    private void getError(double input) {
        error = setpoint - input;

        //wrapping error if input is bounded
        if(inputBounded) {
            double inputRange = maxInput - minInput;
            while (Math.abs(error) > inputRange / 2.0)
                error -= Math.signum(error) * inputRange;
        }
    }

    /**
     * Runs a single iteration of the feedback update with the error calculated by the getError()
     * method, using a trapezoidal sum to approximate the integral of error with respect to time
     * and a difference quotient to approximate the derivative of error with respect to time
     * @param position current measured position (feedback)
     * @param velocity feedforward velocity
     * @param acceleration feedforward acceleration
     * @return the calculated correction
     */
    public double update(double position, double velocity, double acceleration) {
        getError(position);

        double currentTime = System.currentTimeMillis() * 10e3;
        if(Double.isNaN(lastTime)) {
            //special case for handling first iteration
            lastError = error;
            lastTime = currentTime;
            return 0.0;
        }
        //calculating change in time
        double dt = currentTime - lastTime;
        lastTime = currentTime;

        //approximating integral using a trapezoidal sum
        errorSum += 0.5 * (error + lastError) * dt;
        //capping errorSum to prevent integral windup
        errorSum = Range.clip(errorSum, -maxSum, maxSum);
        //approximating derivative using a difference quotient
        double derivative = (error - lastError) / dt;
        lastError = error;

        //calculating output (where e(t) represents the error as a function of time) with kP, kI
        //and kD representing the proportional, integral, and derivative coefficients respectively,
        //v and a representing feedforward velocity and acceleration respectively, kV and kA
        //representing feedforward velocity and acceleration coefficients respectively, and kF(x)
        //being a function of the current position (used for arms and other systems which have
        //other external forces, i.e. gravity.
        //                             t
        //output(t) = kP * e(t) + kI * ∫(e(T))dT + kD * (de(t)/dt - v) + kV * v + kA * a + kF(p)
        //                             0

        double output = pidf.kP * position + pidf.kI * errorSum + pidf.kD * (derivative - velocity)
                + pidf.kV * velocity + pidf.kA * acceleration + pidf.kF.update(position);
        output = MathUtil.approxEquals(output, 0.0) ?
                0.0 : output + Math.signum(output) * pidf.kStatic;

        return outputBounded ? Range.clip(output, minOutput, maxOutput) : output;
    }

    /**
     * Returns true if the error is within the tolerable zone of the input range determined by
     * setTolerance. Assumes that the input bounds were set using the setInputBounds method.
     * @return true if error is tolerable
     */
    public boolean onTarget() {
        if(inputBounded)
            return (Math.abs(error) < tolerance * (maxInput - minInput));
        else
            return false;
    }

    public void reset() {
        errorSum = 0.0;
        lastError = 0.0;
        lastTime = Double.NaN;
    }

    public String telemetryData() {
        return String.format("setpoint: %.2f, error: %.4f, errorSum: %.4f", setpoint, error, errorSum);
    }
}
