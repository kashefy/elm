/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package model.competition;

/**
 *
 * Ornstein-Uhlenbeck (OU) process
 * "Black-box model for inhibition, which inhibits all neurons after one spike, and fires
 * probabilistically according to "membrane potential"
 * Inhibition decays exponentially, excitatory OU noise is added to enable firing" from SEM Matlab code
 * 
 * @author woodstock
 */
import model.neuron.*;
import model.utils.*;
import java.util.Random;

public class WTAOU extends AbstractCompetition{
    
    private double m_dt;        // time resolution
            
    // Parameters for inhibition:
    
    private double m_inhibitionAmplitude;    // Amplitude
    private double m_inhibitionTau;         // Time constant of inhibition
    private double m_inhibitonOffset;       // Offset of inhibition
    
    private double m_inhibition;
    private double [] m_arrInhibitionU;
    
    // Prameters for Ornstein-Uhlenbeck (OU) process:
    
    private double m_ouTau;     // Time constnt of OU process
    private double m_ouSigma;   // Variability of OU process
    private double m_ouMu;      // Offset of OU process
    
    private double m_ou;        // OU Process
    
    private int m_windowSize;   // window allowing simultaneous firing
    
    Random m_generator;
    
    @Override
    public int compete(){
        
        // assuming we already have a reference to prediciton values
        //  Kn = exp(-dt / tau);
        //  params.OU = abs(mu + Kn * (params.OU-mu) + sqrt(1-Kn^2)*randn(1)*sigma*sqrt(tau/2));
        //  params.inhibition = off_inh + (params.inhibition - off_inh) .* exp(-dt / tau_inh);
        
        // revise to minimize repeating the same operations each call
        double kn = Math.exp(-m_dt / m_inhibitionTau);
        m_ou = Math.abs(m_ouMu 
                + kn * (m_ou-m_ouMu) 
                + Math.sqrt(1-kn*kn)*m_generator.nextGaussian()*m_ouSigma*Math.sqrt(m_ouTau/2)
                );
        m_inhibition = m_inhibitonOffset +
                (m_inhibition-m_inhibitonOffset)*Math.exp(-m_dt/m_inhibitionTau);
        
        
        // Compute next spike times for every WTA neuron and choose the earliest firing neuron, if it
        // fires within the next time-step
        
        int nofLearners = m_arrLearners.length;
        m_arrInhibitionU = new double[ nofLearners ];
        
        // computing  effective predictions
        double uEffective;
        double nextSpikeTime;
        //double uInhibition; // inhibition potential
        
        double minVal = Double.MAX_VALUE;
        int minIndex = 0;
        if(nofLearners > 0){
            minVal = m_arrLearners[0].getPredictionValue();
        }
        double nextSpikeTimesSum = 0;
        double [] arrNextSpikeTimes = new double [ nofLearners ]; 
        int wtaIndex;
        double time;
         
        for(int i=0; i<nofLearners; i++){
            
            // u_eff = u - inhibition + OU;
            uEffective = m_arrLearners[i].getPredictionValue() - m_inhibition + m_ou;
            
            // next_spiketimes = exprnd(1./(exp(u_eff)))'; // matlab function takes mean mu=1/lambda as argument
            double lambda = Math.exp(uEffective);
            nextSpikeTime = StdRandom.exp(lambda);

            // params.inh_u = u - inhibition + OU;
            m_arrInhibitionU[i] = uEffective;

            // [m_time, wta_idx] = min(next_spiketimes);
            
            // keep track of smallest nextSpikeTime value to find earliest spike time
            double candidate = nextSpikeTime;
            if(candidate < minVal){

                minVal = candidate;
                minIndex = i;
            }      
            
            nextSpikeTimesSum += nextSpikeTime;
            arrNextSpikeTimes[i] = nextSpikeTime;
        }   
        
        wtaIndex = minIndex;
        time = minVal;
        m_arrOutcome = new int[ nofLearners ]; // equivalent to C in SEM-matlab
        
        //if (sum(next_spiketimes) == 0)
        //  m_time = 0;
        //  if (window_size > 0)
        //    wta_idx = ones(size(C));
        //  else
        //    wta_idx = randint(1,1,[1 length(C)]);
        //  end;
        //end;
        
        // case: everyone wants to spike
        if(nextSpikeTimesSum == 0){
            
            time = 0;
            if(m_windowSize > 0){
                wtaIndex = WTA_ALL;
            }
            else{
                // pick random learner
                wtaIndex = m_generator.nextInt(nofLearners);
            }
        }
        
        // if (m_time < dt)
        if(time < m_dt){
            
        //  if (m_time > 0)
        //    wta_idx = next_spiketimes <= (m_time + window_size);
        //  end;
            
            if(time > 0){
                
                double width = time + m_windowSize;
                int nofHits = 0;
                for(int i=0; i<nofLearners; i++){
                    
                    if(arrNextSpikeTimes[i] <= width){
                        m_arrOutcome[i] = 1;
                        nofHits++;
                    }
                }
                if(nofHits>1)
                    wtaIndex = WTA_MUlTIPLE;
            }
            
            //C(wta_idx) = 1;
            // params.inhibition = Ainh;  % Set inhibition to start point
            m_inhibition = m_inhibitionAmplitude; // maybe redundant to do this.
            
        }
        
        if(wtaIndex >= 0)
            m_arrOutcome[ wtaIndex ] = 1;
        
        return wtaIndex;
    }
    
    public int setParams(CompetitionParams par_params){
        
        m_dt = par_params.get_dt();
        
        setInhibitionParams(par_params.getInhibitionAmplitude(), 
                par_params.getInhibitionTau(), 
                par_params.getInhibitonOffset());
        
        setOUParams(par_params.getOUTau(), 
                par_params.getOUSigma(), 
                par_params.getOUMu());
        
        return 0;
    }
    
    public void setInhibitionParams(double par_inhibitionAmplitude,
            double par_inhibitionTau,
            double par_inhibitonOffset){
        
            m_inhibitionAmplitude   = par_inhibitionAmplitude;
            m_inhibitionTau         = par_inhibitionTau;     
            m_inhibitonOffset       = par_inhibitonOffset;   
       
    }
    
    public void setOUParams(double par_ouTau,
        double par_ouSigma,
        double par_ouMu){
        
        m_ouTau     = par_ouTau;  
        m_ouSigma   = par_ouSigma;
        m_ouMu      = par_ouMu;   
    }
    
    public void init(){
        
       m_ou         = 0;
       m_inhibition = m_inhibitonOffset;
       m_windowSize = 0;
       m_generator  = new Random();
    }
    
    public WTAOU(){
        
    }
}
