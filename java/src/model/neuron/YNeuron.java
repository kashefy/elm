
package model.neuron;

import java.util.Random;

/*
 * class for m_popCodeVal neurons y. Those neurons get their m_popCodeVal (continuous range) [0,1] from the
 * used population code and output spikes based on the m_popCodeVal and their
 * specified firing rate
 */
public class YNeuron 
{
	double m_popCodeVal;     // m_popCodeVal from population code (0 or 1)
	double m_firingProb;        // firing probability for firing rate (r * delta(t))
        double m_deltaT_milSec;
	Random m_generator;           // random number m_generator for equally distributed numbers
	int[] m_arrSpikeHistory;      // history of the spikes(output at time t) of the neuron
	int m_lengthSpikeHistory;        // length of spike history
	
         /*
	 * @param par_nPopCodeInput - m_popCodeVal neuron gets from population code (0 or 1)
	 * @param par_frequency - frequency at which neuron is firing. Values < 0 are equal to infinite firing rate
	 */
        public void init(double par_frequency, double par_deltaT_milSec, int par_lengthSpikeHistory){
	        
            m_deltaT_milSec = par_deltaT_milSec;
            m_lengthSpikeHistory = par_lengthSpikeHistory;
            m_arrSpikeHistory = new int[ m_lengthSpikeHistory ];
            
            //constant for poisson process; m_firingProb = 40Hz * 1ms
            //negative frequencies encode infinite firing rate
            m_firingProb = par_frequency * m_deltaT_milSec;
            m_generator = new Random();

        }
	
	/*
	 * sets the m_popCodeVal allowing range [0,1]
	 */
	public int setPopCodeValue(double par_popCodeVal)
	{
		m_popCodeVal = par_popCodeVal;
		return 0;
	}
	
	/*
	 * returns the state of the neuron at current time step
	 * state is added to spike history
	 */
	public int calcState()
	{
		double s;
		int ret;    // spike or no spike, could turn to boolean..
		
		// non-finite firing rate (frequency < 0 = infinite firing rate)
                // firingProb = f * dT
		if(m_firingProb > 0){
                    
			// no firing at all, if m_popCodeVal is 0
			if(m_popCodeVal == 0){
                            
				ret = 0;
			}
			// for inputs of 1 outputs are generated with f-Hz Poisson process
			else if(m_popCodeVal == 1 || m_popCodeVal == -1)
			{
                            s = m_generator.nextDouble();
                            if(s <= m_firingProb){

                                ret = 1;
                            }
                            else{ 
                                ret = 0;
                            }
			}
                        else{
                            ret =  -1;
			}
			
			// shift spike history to left, m_arrSpikeHistory[m_lengthSpikeHistory-1] is most current
			// time point
			System.arraycopy(m_arrSpikeHistory, 1, m_arrSpikeHistory, 0, m_lengthSpikeHistory-1);
			m_arrSpikeHistory[m_lengthSpikeHistory-1] = ret;
		}
		// infinite firing rate
                else {
                    
                    if(m_popCodeVal == 0) {

                        ret = 0;
                    }
                    // output is always one, no poisson process used
                    else if(m_popCodeVal == 1 || m_popCodeVal == -1) {

                        ret = 1;
                    }
                    else {
                        ret = -1;
                    }
		}
		return ret;
	}
        
        public int[] getHistory(){
            
            int [] histToExport = new int[ m_lengthSpikeHistory ];
            // better return a copy
            System.arraycopy(m_arrSpikeHistory, 0, histToExport, 0, m_lengthSpikeHistory);
            return histToExport;
        }
	
	/*
	 * reset spike history 
	 * used before a new pattern is presented to the network
	 */
	public int resetHistory(){
            
            for(int i = 0; i < m_lengthSpikeHistory; ++i){

                m_arrSpikeHistory[i] = 0;
            }
            return 0;
	}
	
	/*
	 * some test
	 */
	public int test()
	{
            int count = 0;

            for(int i = 0; i < 1000; ++i)
            {
                    count += this.calcState();
            }

            System.out.println("Number of spikes per sec: " + (count));
            return count;
	}
        
        public YNeuron(){
            
	}

}
