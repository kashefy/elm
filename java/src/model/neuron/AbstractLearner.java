/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package model.neuron;

/**
 *
 * @author woodstock
 */
public abstract class AbstractLearner {
    
    protected int m_nofEvidence;
    protected double m_learingRateEtaInitVal;
    protected LearningRate [] m_arrLearningRate;
    protected double m_predictionValue;

    /*
    *  initialize learner parameters
    * @param  - learner parameters
    */  
    public abstract void setParams(LearnerParams params);
    
    /*
    *  initialize learner
    * @param  - 
    */    
    public abstract void init();
    
    /*
    * learn something new
    * @param  - 
    */ 
    public abstract void update();
    
    /*
    * predict response for given input
    * @param par_evidence - array of latest evidence values / window
    */
    
    public abstract double predict(int [] par_evidence);  
    
    //public abstract double predict(int [][] par_evidence); 
        
    public double getLearningRateEtaInitVal() {
        
        return m_learingRateEtaInitVal;
    }

    public void setLearningRateEtaInitVal(double par_learningRateEtaInitVal) {
       
        m_learingRateEtaInitVal = par_learningRateEtaInitVal;
    }
    
    public double [] getLearningRateEta(){
        
        int n = m_nofEvidence;
        double [] arrEta = new double [ n ];
        for(int i=0; i<n; i++){
            
            arrEta[i] = m_arrLearningRate[i].getEta();
        }
        return arrEta;
    }
    
    public double getPredictionValue() {
        return m_predictionValue;
    }

    public void setPredictionValue(double par_predictionValue) {
        this.m_predictionValue = m_predictionValue;
    }
    
    public int getNofEvidence(){
        
        return m_nofEvidence;
    }
    
    /*
     * constructor
     */
    public AbstractLearner(){
        
    }
    
}
