//----------------------------------------------------------------------------
// Copyright (c) FIRST 2008. All Rights Reserved.
// Open Source Software - may be modified and shared by FRC teams. The code
// must be accompanied by the FIRST BSD license file in the root directory of
// the project.
//
// File: FL_PIDController.java
//
// Description: This PIDCOntroller class is a port from the FIRST PID Controller
//              task. This port removes the additional thread overhead but it
//              must run within a periodic task (not a continous task) to work
//              correctly.
//
// Lead: Mark
// ----------------------------------------------------------------------------

package org.usfirst.frc.team1706.robot.utilities;

public class PIDController {

	private double m_P; // factor for "proportional" control
	private double m_I; // factor for "integral" control
	private double m_D; // factor for "derivative" control
	private double m_input; // sensor input for pid controller
	private double m_maximumOutput = 1.0; // |maximum output|
	private double m_minimumOutput = -1.0; // |minimum output|
	private double m_maximumInput = 0.0; // maximum input - limit setpoint to this
	private double m_minimumInput = 0.0; // minimum input - limit setpoint to this
	private boolean m_continuous = false; // do the endpoints wrap around? eg. Absolute encoder
	private boolean m_enabled = false; // is the pid controller enabled
	private double m_prevError = 0.0; // the prior sensor input (used to compute velocity)
	private double m_totalError = 0.0; // the sum of the errors for use in the integral calc
	private double m_tolerance = 0.05; // the percetage error that is considered on target. default 0.05
	private double m_setpoint = 0.0;
	private double m_error = 0.0;
	private double m_result = 0.0;

	/**
	 * Allocate a PID object with the given constants for P, I, D
	 * 
	 * @param Kp
	 *        the proportional coefficient
	 * @param Ki
	 *        the integral coefficient
	 * @param Kd
	 *        the derivative coefficient
	 */
	public PIDController(double Kp, double Ki, double Kd) {

		m_P = Kp;
		m_I = Ki;
		m_D = Kd;

	}

	/**
	 * Read the input, calculate the output accordingly, and write to the output.
	 * This should only be called by the PIDTask
	 * and is created during initialization.
	 */
	private void calculate() {

		// If enabled then proceed into controller calculations
		if (m_enabled) {

			// Calculate the error signal
			m_error = m_setpoint - m_input;

			// !!!!DEBUG!!!
			// System.out.println(m_setpoint);

			// If continuous is set to true allow wrap around
			if (m_continuous) {
				if (Math.abs(m_error) > (m_maximumInput - m_minimumInput) / 2) {
					if (m_error > 0) {
						m_error = m_error - m_maximumInput + m_minimumInput;
					} else {
						m_error = m_error + m_maximumInput - m_minimumInput;
					}
				}
			}

			/*
			 * Integrate the errors as long as the upcoming integrator does
			 * not exceed the minimum and maximum output thresholds
			 */
			if (((m_totalError + m_error) * m_I < m_maximumOutput) && ((m_totalError + m_error) * m_I > m_minimumOutput)) {
				m_totalError += m_error;
			}

			// Perform the primary PID calculation
			m_result = (m_P * m_error + m_I * m_totalError + m_D * (m_error - m_prevError));

			// Set the current error to the previous error for the next cycle
			m_prevError = m_error;

			// Make sure the final result is within bounds
			if (m_result > m_maximumOutput) {
				m_result = m_maximumOutput;
			} else if (m_result < m_minimumOutput) {
				m_result = m_minimumOutput;
			}
		}
	}

	/**
	 * Set the PID Controller gain parameters.
	 * Set the proportional, integral, and differential coefficients.
	 * 
	 * @param p
	 *        Proportional coefficient
	 * @param i
	 *        Integral coefficient
	 * @param d
	 *        Differential coefficient
	 */
	public void setPID(double p, double i, double d) {
		m_P = p;
		m_I = i;
		m_D = d;
	}

	/**
	 * Get the Proportional coefficient
	 * 
	 * @return proportional coefficient
	 */
	public double getP() {
		return m_P;
	}

	/**
	 * Get the Integral coefficient
	 * 
	 * @return integral coefficient
	 */
	public double getI() {
		return m_I;
	}

	/**
	 * Get the Differential coefficient
	 * 
	 * @return differential coefficient
	 */
	public double getD() {
		return m_D;
	}

	/**
	 * Return the current PID result
	 * This is always centered on zero and constrained by the max and min outs
	 * 
	 * @return the latest calculated output
	 */
	public double performPID() {
		calculate();
		return m_result;
	}

	/**
	 * Set the PID controller to consider the input to be continuous,
	 * Rather then using the max and min in as constraints, it considers them to
	 * be the same point and automatically calculates the shortest route to
	 * the setpoint.
	 * 
	 * @param continuous
	 *        Set to true turns on continuous, false turns off continuous
	 */
	public void setContinuous(boolean continuous) {
		m_continuous = continuous;
	}

	/**
	 * Set the PID controller to consider the input to be continuous,
	 * Rather then using the max and min in as constraints, it considers them to
	 * be the same point and automatically calculates the shortest route to
	 * the setpoint.
	 */
	public void setContinuous() {
		this.setContinuous(true);
	}

	/**
	 * Sets the maximum and minimum values expected from the input.
	 *
	 * @param minimumInput
	 *        the minimum value expected from the input
	 * @param maximumInput
	 *        the maximum value expected from the output
	 */
	public void setInputRange(double minimumInput, double maximumInput) {
		m_minimumInput = minimumInput;
		m_maximumInput = maximumInput;
		setSetpoint(m_setpoint);
	}

	/**
	 * Sets the minimum and maximum values to write.
	 *
	 * @param minimumOutput
	 *        the minimum value to write to the output
	 * @param maximumOutput
	 *        the maximum value to write to the output
	 */
	public void setOutputRange(double minimumOutput, double maximumOutput) {
		m_minimumOutput = minimumOutput;
		m_maximumOutput = maximumOutput;
	}

	/**
	 * Set the setpoint for the PIDController
	 * 
	 * @param setpoint
	 *        the desired setpoint
	 */
	public void setSetpoint(double setpoint) {
		if (m_maximumInput > m_minimumInput) {
			if (setpoint > m_maximumInput) {
				m_setpoint = m_maximumInput;
			} else if (setpoint < m_minimumInput) {
				m_setpoint = m_minimumInput;
			} else {
				m_setpoint = setpoint;
			}
		} else {
			m_setpoint = setpoint;
		}
	}

	/**
	 * Returns the current setpoint of the PIDController
	 * 
	 * @return the current setpoint
	 */
	public double getSetpoint() {
		return m_setpoint;
	}

	/**
	 * Retruns the current difference of the input from the setpoint
	 * 
	 * @return the current error
	 */
	public synchronized double getError() {
		return m_error;
	}

	/**
	 * Set the percentage error which is considered tolerable for use with
	 * OnTarget. (Input of 15.0 = 15 percent)
	 * 
	 * @param percent
	 *        error which is tolerable
	 */
	public void setTolerance(double percent) {
		m_tolerance = percent;
	}

	/**
	 * Return true if the error is within the percentage of the total input range,
	 * determined by setTolerance. This asssumes that the maximum and minimum input
	 * were set using setInput.
	 * 
	 * @return true if the error is less than the tolerance
	 */
	public boolean onTarget() {
		return (Math.abs(m_error) < m_tolerance / 100 * (m_maximumInput - m_minimumInput));
	}

	/**
	 * Begin running the PIDController
	 */
	public void enable() {
		m_enabled = true;
	}

	/**
	 * Stop running the PIDController, this sets the output to zero before stopping.
	 */
	public void disable() {
		m_enabled = false;
	}

	/**
	 * Reset the previous error, the integral term, and disable the controller.
	 */
	public void reset() {
		disable();
		m_prevError = 0;
		m_totalError = 0;
		m_result = 0;
	}

	public void setInput(double input) {
		m_input = input;
	}

}
