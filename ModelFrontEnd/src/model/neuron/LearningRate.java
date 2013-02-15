/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package model.neuron;

/**
 *
 * @author woodstock
 */
public class LearningRate {
    
    double m_nEta;
    double m_nEtaLimit;
    double m_nM;        // approximates E[wi]
    double m_nQ;        // approximates E[wi^2]
    
    public void setParams(double par_initEta){
        
        m_nEta = par_initEta;
    }
    
    public void setEtaLimit(double par_nEtaLimit){
        
        m_nEtaLimit = par_nEtaLimit;
    }
    
    public void init(){
        
        m_nM = 0;
        m_nQ = 1;
    }
    
    public void update(double par_input){
        
        //varianceTracking(par_input);
        if(m_nEta > m_nEtaLimit)
            m_nEta = m_nEtaLimit;
    }
    
    private void varianceTracking(double par_w){

        double invEta = 1 - m_nEta;
        double eta_x_w = m_nEta * par_w;
        m_nM = invEta*m_nM + eta_x_w;                           // M_t+1 = (1-eta)*M_t + eta*w
        m_nQ = invEta*m_nQ + eta_x_w*par_w;                     // Q_t+1 = (1-eta)*Q_t + eta*w^2
        
        double var = m_nQ - m_nM*m_nM;
        m_nEta = var/(Math.exp(-m_nM) + 1.0);     // (E[w^2] - E[w]^2)/(e^-E[w]+1)
        //System.out.println(m_nEta);
    }
    
    public double getEta(){
        
        return m_nEta;
    }
    
    public LearningRate(){
        
        m_nEtaLimit = 0.5;
    }
}
