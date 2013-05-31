/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package model.competition;

/**
 *
 * @author woodstock
 */
import java.util.Random;
import model.utils.StdRandom;
import model.utils.ModelUtils;
import org.shared.array.RealArray;

public class WTAPoissonRate extends AbstractCompetition{
    
    private double m_dt;
    private double m_maxRate;
    private double m_lambda;
    private double m_spikeTime;
    
    private Random m_generator;

    public int setParams(CompetitionParams par_params){
        
        setParams(par_params.get_dt(), par_params.getMaxRate());
        return 0;
    }
        
    public void setParams(double par_dt, double par_maxRate){
        
        m_dt        = par_dt;
        m_maxRate   = par_maxRate;
    }
    
    private double computeNextSpikeTime(){
        
        return StdRandom.exp(m_lambda);
    }
    
    @Override
    public int compete(){
        
        // revise to optmize, can do faster, no need to do entire cumsum
        
        //if (next_spiketime < dt)
        //  % Fire now in this interval
        //  K = length(u);
        //  soft_C = exp(u - mean(u));
        //
        //  p = cumsum(soft_C ./ sum(soft_C));
        //  r = rand(1);
        //
        //  C(find(p >= r, 1, 'first')) = 1;
        //  
        //  % Determine next spike
        //  params.next_spiketime = exprnd(1/max_rate);
        //  
        //else
        //  params.next_spiketime = params.next_spiketime - dt;
        int winner_index = -1;
        int nofLearners = m_arrLearners.length;
        m_arrOutcome = new int[ nofLearners ];
        
        if(m_spikeTime < m_dt){
            
            // fire in this interval
            RealArray u = new RealArray(nofLearners);
            RealArray uRel = new RealArray(nofLearners);
            RealArray softMax = new RealArray(nofLearners);
            
            for(int i=0; i<nofLearners; ++i)
                u.set(m_arrLearners[i].getPredictionValue(), i);
            //System.out.printf("u =%n%s%n", u.toString());
            //double um = -u.aMean();
            uRel = u.uAdd(-u.aMean()); // u contains result as well
            //uRel.uAdd(-u.aMax())
            softMax = uRel.uExp().uMul(1./uRel.aSum()); // uRel contains result as well
            
            double r; // draw from an (0,1) interval, uniformly distributed
            do{
                r = StdRandom.uniform(); 
            }while(r==0);// to exclude 0
            
            double [] p = ModelUtils.cumulativeSum(softMax.values(), r);
            
            int i=0;
            while(p[i]<r)
                ++i;
            m_arrOutcome[i] = 1; // the one to fire
            winner_index = i;
            m_spikeTime = computeNextSpikeTime();
        }
        else{
            
            m_spikeTime -= m_dt;
        }
        return winner_index;
    }
    
    @Override
    public void init(){
        
        m_lambda    = 1./m_maxRate;
        m_spikeTime = computeNextSpikeTime();
        m_generator = new Random();
    }

    public WTAPoissonRate() {
        
    }
}
