// PID class to be written by you
public class PID {
	// Current PID parameters
	private PIDParameters p;

    private double I = 0; // Integral part of PID
    private double D = 0; // Derivative part of PID
    private double v = 0; // Computed control signal
    private double e = 0; // Error signal
    private double y = 0; // Measurement signal
    private double yOld = 0; // Old measurement signal
    private double ad; // Help variable for Derivative calculation
    private double bd; // Help variable for Derivative calculation
	
	// Constructor
	public PID(String name) {
        this.p = new PIDParameters();
        p.Beta = 1.0;
		p.H = 0.1;
		p.integratorOn = false;
		p.K = -0.08;  // Default: -0.2
		p.Ti = 0.0;
		p.Tr = 10.0;
        p.Td = 2.0;
        p.N = 10;
		setParameters(p);
    }
	
	// Calculates the control signal v.
	// Called from BeamRegul.
	public synchronized double calculateOutput(double y, double yref) {
        this.y = y;
        this.e = yref - y;

        D = ad * D - bd * (y - yOld);
        this.v = p.K * (p.Beta * yref - y) + I + D;
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
        yOld = y;
    }
	
	// Returns the sampling interval expressed as a long.
	// Note: Explicit type casting needed
	public synchronized long getHMillis() {
        return (long) (p.H * 1000.0);
    }
	
	// Resets integrator and derivative value
	public void reset() {
		this.I = 0.0;
        this.D = 0.0;
	}

	// Sets the PIDParameters.
	// Called from PIDGUI.
	// Must clone newParameters.
	public synchronized void setParameters(PIDParameters newParameters) {
        p = (PIDParameters) newParameters.clone();
        if (! p.integratorOn ) {
            I = 0.0;
        }
        ad = p.Td / (p.Td + p.N * p.H);
        bd = p.K * ad * p.N;
    }

	// Gets the PIDParameters
	// Clones existing parameters
	public synchronized PIDParameters getParameters() {
		return (PIDParameters) p.clone();
	}
}
