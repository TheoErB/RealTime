// PI class to be written by you
public class PI {
	// Current PI parameters
	private PIParameters p;

    private double I = 0; // Integral part of controller
    private double e = 0; // Error signal
    private double v = 0; // Output from controller
	
	// Constructor
	public PI(String name) {
        PIParameters p = new PIParameters();
		p.Beta = 1.0;
		p.H = 0.1;
		p.integratorOn = false;
		p.K = 1.0;
		p.Ti = 0.0;
		p.Tr = 10.0;
		setParameters(p);
    }
	
	// Calculates the control signal v.
	// Called from BeamRegul.
	public synchronized double calculateOutput(double y, double yref) {
        
		this.e = yref - y;
		this.v = p.K * (p.Beta * yref - y) + I;

        return this.v;
    }
	
	// Updates the controller state.
	// Should use tracking-based anti-windup
	// Called from BeamRegul.
	public synchronized void updateState(double u) {
        if (p.integratorOn) {
			this.I = this.I + (p.K * p.H / p.Ti ) * this.e + (p.H / p.Tr) * (u - this.v);
		} else {
			this.I = 0.0;
		}
		
    }
	
	// Returns the sampling interval expressed as a long.
	// Note: Explicit type casting needed
	public synchronized long getHMillis() {

        return (long) (p.H * 1000.0);
    }
	
	// Resets integrator value
	public void reset() {
		this.I = 0.0;
	}

	// Sets the PIParameters.
	// Called from PIGUI.
	// Must clone newParameters.
	public synchronized void setParameters(PIParameters newParameters) {
        p = (PIParameters) newParameters.clone();
		if (! p.integratorOn) {
			I = 0.0;
		}
    }

	// Gets the PIParameters
	// Clones existing parameters
	public synchronized PIParameters getParameters() {
		return (PIParameters) p.clone();
	}
}
