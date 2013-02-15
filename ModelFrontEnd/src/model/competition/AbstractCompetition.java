/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package model.competition;

/**
 *
 * @author woodstock
 */
import model.neuron.*;

public abstract class AbstractCompetition {
    
    public static int WTA_NONE      = -1;
    public static int WTA_MUlTIPLE  = -2;
    public static int WTA_ALL       = -3;

    protected AbstractLearner [] m_arrLearners;
    protected int [] m_arrOutcome;
    
    public int setParams(CompetitionParams par_params){
        
        return 0;
    }

    public void refToLearners(ZNeuron[] par_arrLearners){
        
        m_arrLearners = par_arrLearners;
    }
    
    public int [] getOutcome(){
        
        int n = m_arrOutcome.length;
        int [] arrOutcomeToExport = new int[n];
        System.arraycopy(m_arrOutcome, 0, arrOutcomeToExport, 0, n);
        return arrOutcomeToExport;
    }
    
    public abstract int compete();
    
    public abstract void init();
    
}
