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
import model.utils.*;
import model.utils.files.FileIO;
import org.shared.array.*;

import org.opencv.core.Mat;
import org.opencv.highgui.Highgui;
import org.opencv.core.CvType;

public class FeaturePopulationCode extends SimplePopulationCode{
    
    AbstractFeatureMap m_featureMap;
    ComplexArray [] m_features;
    int [] m_inputDims;
    int [] m_inputComplexDims;
    protected double m_min_response_sum;

    public double get_min_response_sum_val() {
        
        return m_min_response_sum;
    }

    public void set_min_response_sum_val(double par_min_response_sum_val) {
        
        this.m_min_response_sum = par_min_response_sum_val;
    }
    
    private double[] extract_features(double[] par_inputVals){

        // each variable xk is associated
        // with n yi values. n is determined by fanOut value
        double[] features = new double[ m_nofNodes ];
        
        // input Vals -> complex valued stimulus -> response/features
        ComplexArray stimulus;
        double [] stimulusComplexVals = ModelUtils.real2Complex(par_inputVals);
        stimulus = new ComplexArray(stimulusComplexVals, m_inputComplexDims); // works for N-D input
        //m_features =  m_featureMap.convolve(stimulus);
        m_featureMap.setStimulus(stimulus);
        m_features =  m_featureMap.convolve();
        int nof_features = m_features.length;
        
        ComplexArray complexFeatureElements;
        RealArray realFeatureElements;
        int [] feat_dims = m_features[0].dims(); // assumes same equal dims accross features // revise for different scales
        int nof_rows = feat_dims[0];
        int nof_cols = feat_dims[1];
             
        for(int r=0; r<nof_rows; ++r){

            int row_offset = r*nof_cols;
            for(int c=0; c<nof_cols; ++c){

                int [] elementMagnIndex = new int[]{r, c};
                int pos_offset = (row_offset + c)*nof_features;

                for(int fi=0; fi<nof_features; ++fi){

                    // get response magnitude
                    realFeatureElements = m_features[ fi ].torAbs();
                    double element_mag = realFeatureElements.get(elementMagnIndex);
                    features[ pos_offset + fi ] = element_mag;
                }
            }
        }
        return features;
    }
    
    /*
     * @see PopulationCode#getStateOfNeurons(double[])
     */
    @Override
    public double[] calcStateOfNeuronsDistr(double[] par_inputVals) 
    {
        // each variable xk is associated
        // with n*yi values. n is determined by fanOut value
        double [] state_of_neurons_distr_vals = new double[ m_nofNodes ];
        double [] features = extract_features(par_inputVals);
        
        RealArray x = new RealArray(features, features.length);
        x.uMul(1./x.aMax()); // this will directly manipulate double [] features
        
        double [] element_features = new double[ m_features.length ];
        for(int i=0; i<m_nofNodes; i+=m_features.length){

            System.arraycopy(features, i, element_features, 0, m_features.length);
            double elementFeaturesV_sum = new RealArray(element_features, m_features.length).aSum();
            if( elementFeaturesV_sum >= m_min_response_sum ) {
                // significant response to any feature?
                double [] element_distr_vals = eval_distr(element_features);
                System.arraycopy(element_distr_vals, 0, state_of_neurons_distr_vals, i, m_features.length);
            }
        }
//        FileIO.saveArrayToCSV(par_inputVals, 1, par_inputVals.length, "./x/di.csv");
//        FileIO.saveArrayToCSV(features, 1, features.length, "./x/df.csv");
//        FileIO.saveArrayToCSV(state_of_neurons_distr_vals, 1, state_of_neurons_distr_vals.length, "./x/d.csv");
        return state_of_neurons_distr_vals;
    }
    
    /*
     * @see PopulationCode#getStateOfNeurons(double[])
     */
    @Override
    public double[] calcStateOfNeurons(double[] par_inputVals) 
    {
        // each variable xk is associated
        // with n yi values. n is determined by fanOut value
        double[] stateVals = new double[ m_nofNodes ];
        
        // input Vals -> complex valued stimulus -> response/features
        ComplexArray stimulus;
        double [] stimulusComplexVals = ModelUtils.real2Complex(par_inputVals);
        stimulus = new ComplexArray(stimulusComplexVals,m_inputComplexDims); // works for N-D input
        //m_features =  m_featureMap.convolve(stimulus);
        m_featureMap.setStimulus(stimulus);
        m_features =  m_featureMap.convolve();
        
        int i=0;
        
        int nofFeatureSets = m_features.length;
        
        ComplexArray complexFeatureElements;
        RealArray realFeatureElements;
        int [] featureDims = m_features[0].dims();
        int nofRows = featureDims[0];
        int nofCols = featureDims[1];
        int complexDim = featureDims[2];
        int [] elementRealIndex, elementImagIndex, elementMagnIndex;
        double elementReal, elementImag, elementMagn; 
        double [] elementFeatures;
             
        for(int r=0; r<nofRows; ++r){

            int rowOffset = r*nofCols;
            for(int c=0; c<nofCols; ++c){
                
                elementMagnIndex = new int[]{r, c};
                elementFeatures = new double[ nofFeatureSets ];

                for(int featSetIndex=0; featSetIndex<nofFeatureSets; featSetIndex++){
                    
                    // get response magnitude
                    realFeatureElements = m_features[ featSetIndex ].torAbs();
                    elementMagn = realFeatureElements.get(elementMagnIndex);
                    
                    elementFeatures[ featSetIndex ] = elementMagn;                       
                }
               
                double [] elementPopCode = new double[ nofFeatureSets ];
                int maxIndex = encode(elementFeatures, elementPopCode);
//                //i=(a * B * C) + (b * C) + (c * 1)
                
                // Specific case when we have a single 1 for each pop Code, we can perform fewer array access operations
                i = nofFeatureSets*(rowOffset + c) + maxIndex;
                // readjust index to account for bias term if any
                if(m_biasIndex <= i && m_biasIndex >= 0){
                    i++;
                }
                if (elementFeatures[maxIndex] > 0.1);
                    stateVals[i] += 1;
            }
        }
        
        if (m_biasIndex >=0 && par_inputVals.length*nofFeatureSets+1 == m_nofNodes){
            
            stateVals[ m_biasIndex ] = 1; // bias term
        }

        return stateVals;
    }
    
    @Override
    public int [][] sampleStateOfNeurons(double [] par_arrInputDistr, int par_nofSamples){
        // each variable xk is associated
        // with n yi values. n is determined by fanOut value
        
        int[][] stateSamples = new int[ m_nofNodes ][ par_nofSamples ];
        int nofFeatureSets = m_features.length;
        double [] distr = new double[ nofFeatureSets ];
        
        //FileIO.saveArrayToCSV(par_arrInputDistr, 1, par_arrInputDistr.length, "./x/dprs.csv");
        for(int i=0; i<m_nofNodes; i+=nofFeatureSets){
            
            System.arraycopy(par_arrInputDistr, i, distr, 0, nofFeatureSets);
            RealArray distrV = new RealArray(distr, nofFeatureSets);
            double distrV_sumOfAbs = distrV.uAbs().aSum(); // = 0.0 for non-responsive to any feature
            if(distrV_sumOfAbs > 0.0){
            
                int [] samples_subset = sample(distr, par_nofSamples);
                for(int j=0; j<par_nofSamples; ++j){
                    stateSamples[ i + samples_subset[j] ][j] = 1;
                }   
            }
        }
        //FileIO.saveArrayToCSV(stateSamples, ".\\x\\sp.csv");
        return stateSamples;
    }
    
    public void setFeatureMap(AbstractFeatureMap par_featureMap){
        
//        if(m_featureMap != null){
//            
//            m_featureMap = par_featureMap;
//            if(m_biasValue != 0){
//
//                m_nofNodes = (m_nofNodes-1)*m_featureMap.getNofFeatureSets()+1;
//                m_biasIndex = m_nofNodes;
//            }
//            else{
//                m_nofNodes *= m_featureMap.getNofFeatureSets();
//                m_biasIndex = -1;
//            }
//        }
         m_featureMap = par_featureMap;
    }
    
    public void setInputDimensions(int [] par_dims){
        
        m_inputDims = new int [ par_dims.length ];
        System.arraycopy(par_dims, 0, m_inputDims, 0, par_dims.length);
        
        m_inputComplexDims = new int [ m_inputDims.length+1 ];
        System.arraycopy(m_inputDims, 0, m_inputComplexDims, 0, m_inputDims.length);
        m_inputComplexDims[ m_inputComplexDims.length-1 ] = 2;   
    }
    
    // select strongest to activate and shut down the rest
    // output nodes per input node are antognistic
    // return index of strongest popCode for input set
    private int encode(double [] par_inputs, double [] par_elementPopCode){
        
//        for(int t=0; t<par_inputs.length; t++)
//            System.out.print(par_inputs[t]+" ");
//        System.out.println();
        return encodeWTA(par_inputs, par_elementPopCode);
    }
    
    private int encodeWTA(double [] par_inputs, double [] par_elementPopCode){
        
        double [] popCode = par_elementPopCode;
        
        int nofInputs = par_inputs.length;
        int nofNodes = nofInputs;
        //popCode = new double[ nofNodes ];

        // j: index of input AND output nodes
        // select strongest to activate and shut down the rest
        // output nodes per input node are antognistic

        int j=0;
        int boundIndex = nofNodes;
        int maxIndex = 0;
        double maxVal = par_inputs[j++]; //-1;
        for(; j<boundIndex; ++j){

            //double candidate = Math.abs(par_inputs[j]);
            double candidate = par_inputs[j];
            if(candidate > maxVal){

                maxVal = candidate;
                maxIndex = j;
            }
        }
        popCode[ maxIndex ] = 1.0;
        return maxIndex;
    }
    
    private static double [] eval_distr(double [] par_inputs){
        
        int n = par_inputs.length;
        double [] inputs = new double[n];
        System.arraycopy(par_inputs, 0, inputs, 0, n);
        
        RealArray soft_max = new RealArray(inputs, n);
        
        soft_max = soft_max.uMul(1./soft_max.aSum());
        soft_max = soft_max.uExp();      // to handle negative and zero values 
        soft_max = soft_max.uPow(10);    // to suppress values < 1.0
        //soft_max = soft_max.uAdd(1.0);   // to handle zero values   
        double sum = soft_max.aSum();
        
        soft_max = soft_max.uMul(1./sum);  
            
        return soft_max.values();
    }
    
    private static int [] sample(double [] par_distribution, int nof_samples){
              
        int [] samples = new int [ nof_samples ];
        double [] p = ModelUtils.cumulativeSum(par_distribution);
        
//        for(int j=0; j<nofInputs; j++){
//            
//            System.out.print(soft_max.values()[j] + " ");
//
//        }
//        System.out.println();
        
        for(int t=0; t<nof_samples; ++t){
            
            double r = StdRandom.uniform();
            int j=0;
            while(p[j]<r){
                ++j;
            }
            samples[t] = j;
        }

        return samples;
    }
    
    private static int encodeViaProb(double [] par_inputs, double [] par_elementPopCode){
        
        double [] popCode = par_elementPopCode;
        int nofInputs = par_inputs.length;
        double [] inputs = new double[ nofInputs ];
        System.arraycopy(par_inputs, 0, inputs, 0, nofInputs);
        
        RealArray softMax = new RealArray(inputs,nofInputs); 
        softMax = softMax.uExp();      // to handle negative values
        softMax = softMax.uPow(10);    // to suppress values < 1.0
        softMax = softMax.uAdd(1.0);   // to handle zero values   
        double sum = softMax.aSum();
        
        softMax = softMax.uMul(1.0/sum);  
        
        double [] p = ModelUtils.cumulativeSum(softMax.values());
        
//        for(int j=0; j<nofInputs; j++){
//            
//            System.out.print(soft_max.values()[j] + " ");
//
//        }
//        System.out.println();
        
        double r = StdRandom.uniform();
        int i=0;
        while(p[i]<r){
            ++i;
        }

        popCode[ i ] = 1.0;
        return i;
    }
    
    public static void testEncodeViaProb(){
        
        int nofTrials = 100000;
        int nofNodes = 6;
        double[] arrInputs = new double[ nofNodes ];
        
        // set values
        for(int i=0; i<nofNodes; ++i){
            
            arrInputs[i] = StdRandom.random();
        }
        
        int [] arrCount = new int [ nofNodes ];
        for(int ti=0; ti<nofTrials; ++ti){
            
            double[] popCode = new double [nofNodes];
            int wIndex = encodeViaProb(arrInputs, popCode);
            arrCount[wIndex]++;
        }
        
        for(int i=0; i<nofNodes; ++i){

            System.out.print(arrInputs[i] +"  ");
                
         }
        System.out.println();
                
        for(int i=0; i<nofNodes; ++i){

            System.out.print(arrCount[i] +"  ");
                
         }
        System.out.println();
    }
    
    public FeaturePopulationCode(){
        
        m_min_response_sum = 3e-0;//Double.NEGATIVE_INFINITY;
    }
}
