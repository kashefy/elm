/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package model.encoding;

/**
 *
 * @author woodstock
 */
import model.features.AbstractFeatureMap;
import model.neuron.*;

public class Encoder extends AbstractEncoder{
    
    protected double m_frequency;
    protected double m_deltaT_sec;
    protected double m_deltaT_milSec;
    protected int m_durationInMilSec;
    
    protected AbstractPopulationCode popCoder;
    protected int m_nofPopCodeInputs;
    protected int m_nPopCodeFanOut;
    protected double [] m_popCodeStateVals;
    protected int [] m_arr_input_dimensions;
    
    protected YNeuron[] m_arrSpikingNode;
    
    // feature members
    protected AbstractFeatureMap m_featureMap;
    
    @Override
    public void set_params(EncoderParams params){
        
        this.setFeatureMap(params.get_feature_map());
        m_frequency = params.get_frequency();
        m_deltaT_sec = params.get_delta_t();
        m_deltaT_milSec = m_deltaT_sec*1000;
        m_durationInMilSec = params.get_duration_milSec();
        m_nofPopCodeInputs = params.get_nof_pop_code_inputs();
        m_nPopCodeFanOut = params.get_pop_code_fan_out();
        m_arr_input_dimensions = params.get_arr_input_dims();
    }
    
    @Override
    public void init(){
        
        if(m_featureMap != null){
           
            popCoder = new FeaturePopulationCode();
            FeaturePopulationCode featPopCoder = (FeaturePopulationCode)popCoder;
            featPopCoder.setFeatureMap(m_featureMap);
            featPopCoder.setInputDimensions(m_arr_input_dimensions);
            featPopCoder.set_min_response_sum_val(0.01);
        }
        else{
           popCoder = new SimplePopulationCode(); 
        }

        popCoder.init(m_nofPopCodeInputs, m_nPopCodeFanOut);
        
        int nofNodes = popCoder.getnofNodes();
        m_arrSpikingNode = new YNeuron[ nofNodes ];
        for(int i=0; i<nofNodes; i++){
            
            m_arrSpikingNode[i] = new YNeuron();
            m_arrSpikingNode[i].init(m_frequency,m_deltaT_sec,(int)m_deltaT_milSec*m_durationInMilSec);
        } 
    }
        
    public void init(double par_frequency, double par_deltaT, 
            int par_duration,
            int par_nofPopCodeInputs,
            int par_nPopCodeFanOut){
        
        m_frequency = par_frequency;
        m_deltaT_sec    = par_deltaT;
        m_deltaT_milSec = m_deltaT_sec*1000;
        m_durationInMilSec  = par_duration;
        
        m_nofPopCodeInputs = par_nofPopCodeInputs;
        m_nPopCodeFanOut = par_nPopCodeFanOut;
        
        if(m_featureMap != null){
           
            popCoder = new FeaturePopulationCode();
            FeaturePopulationCode featPopCoder = (FeaturePopulationCode)popCoder;
            featPopCoder.setFeatureMap(m_featureMap);
        
        }
        else{
           popCoder = new SimplePopulationCode(); 
        }

        popCoder.init(m_nofPopCodeInputs, m_nPopCodeFanOut);
        
        int nofNodes = popCoder.getnofNodes();
        m_arrSpikingNode = new YNeuron[ nofNodes ];
        for(int i=0; i<nofNodes; i++){
            
            m_arrSpikingNode[i] = new YNeuron();
            m_arrSpikingNode[i].init(m_frequency,m_deltaT_sec,(int)m_deltaT_milSec*m_durationInMilSec);
        }         
    }
    
    public void setFeatureMap(AbstractFeatureMap par_featureMap){
        
        m_featureMap = par_featureMap;
    }
    
    public void setInputDimensions(int [] par_arrDimensions){
        
        FeaturePopulationCode featPopCoder = (FeaturePopulationCode)popCoder;
        featPopCoder.setInputDimensions(par_arrDimensions);
    }
    
    /**
     * @brief generate population code
     * @param input Values
     * @return population code
     */
    public double[] genPopCode(double [] par_inputVals){
        
        double [] popCodeStateVals;
        popCodeStateVals = popCoder.calcStateOfNeurons(par_inputVals);

        //be extra cautious and create an explicit copy
        m_popCodeStateVals = new double[ popCodeStateVals.length ];
        System.arraycopy(popCodeStateVals, 0, m_popCodeStateVals, 0, popCodeStateVals.length);
        return popCodeStateVals;
    }
    
    /**
     * @brief generate spike trains from pre-generated population code
     * @param par_inputVals
     * @return spike trains [no. of nodes][no. of spikes per node]
     */
    public int[][] genSpikeTrains(double [] par_inputVals){
        
        int nofNodes = popCoder.getnofNodes();
        int nof_spikes_per_node = (int)m_deltaT_milSec*m_durationInMilSec;
        int [][] spikeTrains = new int[ nofNodes ][ nof_spikes_per_node ];
                     
        for (int yi=0; yi<nofNodes; yi++){

            m_arrSpikingNode[yi].setPopCodeValue(m_popCodeStateVals[yi]);
            // generate spike train for each spiking node
            for (int t=0; t<nof_spikes_per_node; t++){
               
               m_arrSpikingNode[yi].calcState();
            }
            
            // accumulate spikes for each node
            spikeTrains[yi] = m_arrSpikingNode[yi].getHistory();  
        }
        return spikeTrains;
    }
    
    public int[][] encode(double [] par_inputVals){
        
        genPopCode(par_inputVals);
        int [][] spikeTrains = genSpikeTrains(par_inputVals);
        return spikeTrains;
    }
    
    public double [] getPopCode(){
        
        double [] popCodeStateVals;
        //be extra cautious and create an explicit copy
        popCodeStateVals = new double[ m_popCodeStateVals.length ];
        System.arraycopy(m_popCodeStateVals, 0, popCodeStateVals, 0, m_popCodeStateVals.length);
        return popCodeStateVals;
    }
    
    public int getNofEncoderNodes(){
        
        return popCoder.getnofNodes();
    }
    
    public Encoder(){
            
    }
}
