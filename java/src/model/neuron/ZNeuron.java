/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package model.neuron;

/**
 *
 * @author woodstock
 */
import java.util.Random;

public class ZNeuron extends AbstractLearner{
    
    // weight members
    protected boolean m_bInitRandWeights;
    protected double m_scaleOfRandWeights;
    protected double[] m_initWeights; // excludes bias term
    protected double[] m_weights; // excludes bias term
    protected double m_nWeightLimit;
    
    // bias members
    private double m_nBias;
    private LearningRate m_biasLearningRate;
    
    // history members
    protected int m_historyLength;
    private int [][] m_spikeHistory;
    
    // response members
    protected double m_membranePotential;
    protected boolean m_bFiredRecently;
    
    @Override
    public void setParams(LearnerParams params){
        
        setHistoryLength(params.getHistoryLength());
        m_nofEvidence = params.getNofInputNodes();
        // nof Evidence excludes bias term
        m_bInitRandWeights = params.initWithRandomWeights();
        
        if(m_bInitRandWeights){
            
            m_scaleOfRandWeights = params.getScaleOfRandWeights();
        }
        else{
            
            double [] weightsRef = params.getWeights();
            if(weightsRef != null){
                
                if(params.wasBiasSet()){

                    setInitialweights(weightsRef,params.getBias());

                }
                else{
                    setInitialweights(weightsRef); // and use default bias value.
                }
            }
        }
        //m_nWeightLimit = params.getWeightLimit();
        
        setLearningRateEtaInitVal(params.getLearningRateEtaInitVal()); 
        m_biasLearningRate = new LearningRate();
        m_biasLearningRate.setParams(m_learingRateEtaInitVal);
        m_arrLearningRate = new LearningRate[m_nofEvidence];
        for(int i=0; i<m_nofEvidence; ++i){
            
            m_arrLearningRate[i] = new LearningRate();
            m_arrLearningRate[i].setParams(m_learingRateEtaInitVal);
        }
    }

    /*
    *  initialize learner parameters
    * @param  - 
    */        
    @Override
    public void init(){
        
        initHistory();
        if(m_bInitRandWeights){
            
            setRandomWeights(m_scaleOfRandWeights);
        }
        // otherwise already set arbitrarily in setParams()
        
        // init learning rate
        m_biasLearningRate.init();
        for(int i=0; i<m_nofEvidence; ++i){
            
            m_arrLearningRate[i].init();
        }
        
    }
    
    public void setBias(double par_nBias){
        
        m_nBias = par_nBias;
    }
    
    public double getBias(){
        
        return m_nBias;
    }
    
    public double getBiasLearningRate(){
        
        return m_biasLearningRate.getEta();
    }
    
    public void setInitialweights(double [] par_initialWeights){
        
        m_weights = new double [ m_nofEvidence ];
        if(par_initialWeights.length == m_nofEvidence+1){
            
            m_nBias = par_initialWeights[0];
            // make own copy
            System.arraycopy(par_initialWeights, 1, m_weights, 0, par_initialWeights.length);   
        }
        else{
            
            // use default value for bias
            // make own copy
            System.arraycopy(par_initialWeights, 0, m_weights, 0, par_initialWeights.length);
        }
        keepInitialWeights(m_weights);
    }
    
    public int setInitialweights(double [] par_initialWeights, double par_nBias){
        
        m_weights = new double [ m_nofEvidence ];
        if(par_initialWeights.length == m_nofEvidence){
            
            m_nBias = par_nBias;
            // make own copy
            System.arraycopy(par_initialWeights, 0, m_weights, 0, m_nofEvidence);
            keepInitialWeights(m_weights);
            return 0;
            
        }
        else{
           
            //String message = "unexpected number of weights in weight vector inspite of bias provided explicitly";
            //Exception err = new Exception(message);
            
            //throw(err);
            return 1;
        }
    }
    
    public void setRandomWeights(double par_scale){
        
        m_weights = new double [ m_nofEvidence ];
        m_initWeights = new double [ m_nofEvidence ];
        Random generator = new Random();

        m_nBias = generator.nextGaussian();
        if(m_nBias > 0)
            m_nBias = -m_nBias; // keep all negative

        for(int wi = 0; wi < m_nofEvidence; ++wi){
            
            double weight = generator.nextGaussian();
            if(weight > 0)
                weight = -weight; // keep all negative
            
            m_weights[wi] = weight * par_scale;
        }   
    }
    
    private void keepInitialWeights(double [] par_weights){
        
        int nofWeights = par_weights.length;
        m_initWeights = new double[ nofWeights ];
        System.arraycopy(par_weights, 0, m_initWeights, 0, nofWeights);
    }
    
    public void setWeights(double[] par_arrWeightVals){
        
        System.arraycopy(par_arrWeightVals, 0, m_weights, 0, m_nofEvidence);
    }
    
    public double[] getWeights(){
        
        double [] weightsToExport = new double [ m_nofEvidence ];
        // make explicit copy
        System.arraycopy(m_weights, 0, weightsToExport, 0, m_nofEvidence);
        return weightsToExport;
    }
    
    public double getWeights(double[] par_weightsToExport){
        
        // make explicit copy
        System.arraycopy(m_weights, 0, par_weightsToExport, 0, m_nofEvidence);
        return m_nBias;
    }
    
    public void setHistoryLength(int par_historyLength){
        
        m_historyLength = par_historyLength;
    }

    /*
     * learn something new
     * Updates weights with unsupervised Bayesian Hebb rule
     * @param  x ... new training example vector
     * @param C ... vector of cluster activations
    */ 
    @Override
    public void update(){
        
        double limitFactor;
        double deltaW, wOld, w0, w;
        double nEta;                 // learning rate
                   
        // determine limit factor
        //    old_w0 = bh.w(1,i);
        //    limit_factor = exp(-max(old_w0, log(nEta)));
        wOld = m_nBias;
        nEta = m_biasLearningRate.getEta();
        limitFactor = Math.exp( -Math.max(wOld, Math.log(nEta)));

        // update w0
        // bh.w(1,i) = old_w0 + nEta * (C(i) * (1 - exp(old_w0)) - (1-C(i)) * exp(old_w0)) * ...
            // limit_factor;    
        // Nessler's (2010) STDP equation (12)

        deltaW = nEta * limitFactor;
        deltaW *=  (m_bFiredRecently)? (1-Math.exp(wOld)) : -(Math.exp(wOld));
        w0 = wOld + deltaW;

        // bh.w(1,i) = max(bh.w(1,i), -bh.limit);
        m_nBias = Math.max(w0, -m_nWeightLimit);
        m_biasLearningRate.update(m_nBias);         // adaptive learning rate

        // update all other weights
        if(m_bFiredRecently){

            for(int wi=0; wi<m_nofEvidence; ++wi){

                wOld = m_weights[ wi ];
                nEta = m_arrLearningRate[wi].getEta();
                // limit_factor = exp(-max(old_w, log(nEta)));
                limitFactor = Math.exp( -Math.max(wOld, Math.log(nEta)) );

                // determine if input node has fired recently

//                    int nInputFiredRecently = 0;
//                    int t = m_historyLength-1;
//
//                    while(nInputFiredRecently == 0 && t >= 0){
//
//                        nInputFiredRecently += m_spikeHistory[wi][t];
//                        t--;
//                    }

                int nInputFiredRecently = hasInputFiredRecently(wi);

//                    delta = nEta .* (x .* (1 - exp(old_w)) - (1-x) .* ...
//                                      exp(old_w));

                deltaW = nEta * limitFactor;
                deltaW *= (nInputFiredRecently > 0)? 1-Math.exp(wOld) : -Math.exp(wOld);

//                    bh.w(2:(bh.dim+1),i) = old_w + delta .* C(i) .* limit_factor;
//                    bh.w(2:(bh.dim+1),i) = max(bh.w(2:(bh.dim+1),i), -bh.limit);   

                w = wOld + deltaW;
                w = Math.max(w, -m_nWeightLimit);
                m_weights[ wi ] = w;
                m_arrLearningRate[wi].update(w);         // adaptive learning rate
            }
        }
    }
    
    /*
    * predict response for given input with Bayesian-Hebb rule
    * @param  par_spikes - vector indicating which YNeurons spiked
    * @return membrane potential
    */
    @Override    
    public double predict(int [] par_spikes){

        double nMembranePotential = 0.0;
        
        // advance and update history
        this.advanceHistory(1);
        updateHistory(par_spikes);
            
        // u = bh.w' * [x];
        nMembranePotential += m_nBias;
        for(int yi=0; yi<m_nofEvidence; ++yi){

            int nInputFiredRecently = hasInputFiredRecently(yi);
            if(nInputFiredRecently > 0){

                nMembranePotential += m_weights[yi];
            }
        }
        m_membranePotential = nMembranePotential;
        
        return m_membranePotential;
    }
    
    protected int hasInputFiredRecently(int par_inputNodeIndex){
       
        int nInputFiredRecently = 0;
        int t = m_historyLength-1;

        while(nInputFiredRecently == 0 && t >= 0){

            nInputFiredRecently += m_spikeHistory[ par_inputNodeIndex ][t];
            --t;
        }
        return nInputFiredRecently;
    }
    
    protected void initHistory(){
        
        m_spikeHistory = new int[ m_nofEvidence ][ m_historyLength ];
    }
    
    protected void updateHistory(int [] par_spikes){
        
        for(int yi=0; yi<m_nofEvidence; ++yi){
            
            m_spikeHistory[yi][ m_historyLength-1 ] = par_spikes[yi];
        
        }
    }
    
    public void resetHistory(){
        
        m_spikeHistory = new int[ m_nofEvidence ][ m_historyLength ];
    }
    
    /*
    * padvance spike history by n time steps
    * @param  par_nSteps - number of tie stemps to advanceHistory in
    */
    protected void advanceHistory(int par_nSteps){
        
        // shift spike history to left, m_arrSpikeHistory[m_lengthSpikeHistory-1] is most current
	// time point
        // perform for all nodes
        for(int yi=0; yi<m_nofEvidence; ++yi){
            
            for (int i=0; i<par_nSteps; ++i){

                System.arraycopy(m_spikeHistory[yi], 1, m_spikeHistory[yi], 0, m_historyLength-1);
                m_spikeHistory[yi][ m_historyLength-1 ] = -1;
            }
        }
    }
    
    public void letFire(boolean par_bfiringState){
        
        m_bFiredRecently = par_bfiringState;
    }
    
    @Override
    public double getPredictionValue() {
        
        m_predictionValue = m_membranePotential;
        return m_predictionValue;
    }
    
    /*
    * constructor
    */
    public ZNeuron(){
        
        m_historyLength = 1;
        m_nBias = 0.0;  // default bias value => no bias
        m_bFiredRecently = false;
        m_nWeightLimit = Double.MAX_VALUE;
    }
}
