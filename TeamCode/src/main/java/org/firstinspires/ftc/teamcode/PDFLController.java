package org.firstinspires.ftc.teamcode;

public class PDFLController {
    private double kP, kD, kF, kL;
    private double lastTimeStamp;
    private double period;
    private double prevErrorVal;
    private double kL_tolerance = 0;

    /**
     * @param kP This should be the third number you tune. Increase until you reach your desired
     *           set point.
     * @param kD This is the dampening coefficient. Tune this last by increasing the value until you
     *          remove the oscillation from the system (if your system is still oscillating, lower kP)
     * @param kF This is the feedforward component and the first value to tune. Use this to counteract
     *           forces (i.e. gravity) in your system. Tune by increasing until system no longer
     *           moves because of the force.
     * @param kL This is the lower-limit component. Tune it second by increasing until your system
     *           starts to move, then back it off until it barely doesn't move anymore.
     */
    public PDFLController(double kP, double kD, double kF, double kL) {
        lastTimeStamp = 0;
        period = 0;
        this.kP = kP;
        this.kD = kD;
        this.kF = kF;
        this.kL = kL;
    }
    /**
     * @param kP This should be the third number you tune. Increase until you reach your desired
     *           set point.
     * @param kD This is the dampening coefficient. Tune this last by increasing the value until you
     *          remove the oscillation from the system (if your system is still oscillating, lower kP)
     * @param kF This is the feedforward value and the first val to tune. Use this to counteract
     *           forces (i.e. gravity) in your system. Tune by increasing until system no longer
     *           moves because of the force.
     * @param kL This is the lower-limit component. Tune it second by increasing until your system
     *           starts to move, then back it off until it barely doesn't move anymore.
     * @param kL_tolerance Add this in if your system is experiencing jittering when it reaches the
     *                     set point. Increase slightly until the jittering stops.
     */
    public PDFLController(double kP, double kD, double kF, double kL, double kL_tolerance) {
        this(kP, kD, kF, kL);
        this.kL_tolerance = kL_tolerance;
    }

    /**
     * This allows you to set your PDFL constant easily. This might be necessary if your
     * feedforward changes as your system moves (i.e. in the case of a rotating arm, you might have
     * to adjust your power as a function of the cosine of your angle).
     */
    public void setPDFLConstants(double kP, double kD, double kF, double kL) {
        this.kP = kP;
        this.kD = kD;
        this.kF = kF;
        this.kL = kL;
    }

    public void reset() {
        prevErrorVal = 0;
        lastTimeStamp = 0;
    }

    /**
     * @param error The error of the system (usually target - measured value)
     * @return The computed output power
     */
    public double calculate(double error) {
        double p = pComponent(error);
        double d = dComponent(error);
        double f = fComponent();
        double l = lComponent(error);
        return p + d + f + l;
    }

    // Computes a response power proportional to the error
    private double pComponent(double error) {
        return kP * error;
    }


     // Computes the derivative component of the PDFL controller. (courtesy of FTC Lib)
     //
     // The derivative term helps predict future error by calculating the rate of
     // change of the error over time. This helps dampen oscillations and improve
     // system stability by counteracting rapid changes.

     // The calculation follows these steps:
     // - Determine the time elapsed (`period`) since the last call.
     // - Compute the rate of change of error (`errorVal_v`), ensuring no division by an
     //   zero to avoid instability.
     // - Multiply by the derivative gain (`kD`) to scale the contribution appropriately.
    private double dComponent(double error) {
        double errorVal_v;
        double currentTimeStamp = (double) System.nanoTime() / 1E9;

        if (lastTimeStamp == 0) lastTimeStamp = currentTimeStamp;
        period = currentTimeStamp - lastTimeStamp;
        lastTimeStamp = currentTimeStamp;
        if (Math.abs(period) > 1E-6) {
            errorVal_v = (error - prevErrorVal) / period;
        } else {
            errorVal_v = 0;
        }
        prevErrorVal = error;
        return kD * errorVal_v;
    }

    // Returns a constant power to be applied to counteract a force
    private double fComponent() {
        return kF;
    }

     // This component applies a fixed control response based on error magnitude to counteract
     // friction, similar to a Bang-Bang controller but with a tolerance range where no
     // correction is applied. This can help prevent unnecessary jittering
     // around the set point.
     //
     // The control logic:
     // - If the absolute error is within `kL_tolerance`, return 0 (no response).
     // - Otherwise, apply a fixed correction (`kL`) in the direction of the error.
    private double lComponent(double error) {
        double response = 0;
        if (Math.abs(error) < kL_tolerance) {
            response = 0;
        } else {
            response = Math.signum(error) * kL;
        }
        return response;
    }
}