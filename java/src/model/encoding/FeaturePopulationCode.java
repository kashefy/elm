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
import org.shared.array.*;

public class FeaturePopulationCode extends SimplePopulationCode{
    
    AbstractFeatureMap m_featureMap;
    ComplexArray [] m_features;
    int [] m_inputDims;
    int [] m_inputComplexDims;
    protected double m_min_response_sum_val;

    public double get_min_response_sum_val() {
        
        return m_min_response_sum_val;
    }

    public void set_min_response_sum_val(double par_min_response_sum_val) {
        
        this.m_min_response_sum_val = par_min_response_sum_val;
    }
    
    private double[] extractFeatures(double[] par_inputVals){

        // each variable xk is associated
        // with n yi values. n is determined by fanOut value
        double[] features = new double[ m_nofNodes ];
        
        // input Vals -> complex valued stimulus -> response/features
        ComplexArray stimulus;
        double [] stimulusComplexVals = ModelUtils.prepRealValuesForComplex(par_inputVals);
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
             
        for(int r=0; r<nofRows; r++){

            int rowOffset = r*nofCols;
            for(int c=0; c<nofCols; c++){

                //elementRealIndex = new int[]{r,c,0};
                //elementImagIndex = new int[]{r,c,1};
                elementMagnIndex = new int[]{r,c};

                elementFeatures = new double[ nofFeatureSets ];
                
                int posOffset = (rowOffset + c)*nofFeatureSets;

                for(int fi=0; fi<nofFeatureSets; fi++){

                    // get real and imaginary response components separately
                    //featureElements = m_features[ fi ];
                    //elementReal = featureElements.get(elementRealIndex);
                    //elementImag = featureElements.get(elementImagIndex);
                    
                    // get response magnitude
                    realFeatureElements = m_features[ fi ].torAbs();
                    elementMagn = realFeatureElements.get(elementMagnIndex);
                    
                    elementFeatures[ fi ] = elementMagn;    
                    features[ posOffset + fi ] = elementMagn;
                }
            }
        }
        
//        if (m_biasIndex >=0 && par_inputVals.length*nofFeatureSets+1 == m_nofNodes){
//
//            features[ m_biasIndex ] = 1; // bias term
//        }

        return features;
    }
    
    /*
     * @see PopulationCode#getStateOfNeurons(double[])
     */
    @Override
    public double[] calcStateOfNeuronsDistr(double[] par_inputVals) 
    {
        // each variable xk is associated
        // with n yi values. n is determined by fanOut value
        double[] stateOfNeuronsDistrVals = new double[ m_nofNodes ];
        
        double[] features = extractFeatures(par_inputVals);
        
        int nofFeatureSets = m_features.length;
        double [] elementFeatures = new double[ nofFeatureSets ];
        double [] no_response = new double[ nofFeatureSets ];
             
        for(int i=0; i<m_nofNodes; i+=nofFeatureSets){

            System.arraycopy(features, i, elementFeatures, 0, nofFeatureSets);
            //System.arraycopy(features, t, stateOfNeuronsDistrVals, t, nofFeatureSets);
            
//                        if(i==2958){
//                            System.out.println();  
//                 for(int fi=0; fi<nofFeatureSets; fi++){
//
//                    System.out.print(elementFeatures[ fi ] +" ");    
//
//                }                     
//                System.out.println();    
//                
//            }
            double [] singleDistr;
            RealArray elementFeaturesV = new RealArray(elementFeatures, nofFeatureSets);
            double elementFeaturesV_sum = elementFeaturesV.aSum();
            if( elementFeaturesV_sum < m_min_response_sum_val ) {
                // no significant response to any feature 
                singleDistr = no_response;
            }
            else{
                singleDistr = evalDistr(elementFeatures);
            }
            System.arraycopy(singleDistr, 0, stateOfNeuronsDistrVals, i, nofFeatureSets);
//            if(t==2958){
//                        
//                System.out.println();
//                for(int fi=0; fi<nofFeatureSets; fi++){
//                    System.out.print(elementFeatures[fi]+" ");
//                }
//                System.out.println();
//                System.out.print("r: ");
//                evalDistr(elementFeatures);
//
//                System.out.print("e: ");
//                for(int j=0; j<50; j++){
//                        int m = encodeViaProb(elementFeatures, new double[nofFeatureSets]);
//                        System.out.print(m+ " ");
//                }
//                System.out.println();
//            }
        }
        return stateOfNeuronsDistrVals;
    }
    
    /*
     * @see PopulationCode#getStateOfNeurons(double[])
     */
    @Override
    double[] calcStateOfNeurons(double[] par_inputVals) 
    {
        // each variable xk is associated
        // with n yi values. n is determined by fanOut value
        double[] stateVals = new double[ m_nofNodes ];
        
        // input Vals -> complex valued stimulus -> response/features
        ComplexArray stimulus;
        double [] stimulusComplexVals = ModelUtils.prepRealValuesForComplex(par_inputVals);
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
             
        for(int r=0; r<nofRows; r++){

            int rowOffset = r*nofCols;
            for(int c=0; c<nofCols; c++){

                //elementRealIndex = new int[]{r,c,0};
                //elementImagIndex = new int[]{r,c,1};
                elementMagnIndex = new int[]{r,c};

                elementFeatures = new double[ nofFeatureSets ];

                for(int featSetIndex=0; featSetIndex<nofFeatureSets; featSetIndex++){

                    // get real and imaginary response components separately
                    //featureElements = m_features[ fi ];
                    //elementReal = featureElements.get(elementRealIndex);
                    //elementImag = featureElements.get(elementImagIndex);
                    
                    // get response magnitude
                    realFeatureElements = m_features[ featSetIndex ].torAbs();
                    elementMagn = realFeatureElements.get(elementMagnIndex);
                    
                    elementFeatures[ featSetIndex ] = elementMagn;                       
                }
               
                double [] elementPopCode = new double[ nofFeatureSets ];
                int maxIndex = encode(elementFeatures, elementPopCode);
//                for(int z=0;z<elementFeatures.length;z++){
//                    System.out.print(elementFeatures[z]+" ");
//                }
//                System.out.println();
//                for(int z=0;z<elementPopCode.length;z++){
//                    System.out.print(elementPopCode[z]+" ");
//                }
//                System.out.println();
                
//                // General case: store popCode into stateVal by array copy
//                //i=(a * B * C) + (b * C) + (c * 1)
//                t = nofFeatureSets*(rowOffset + c);
//                //System.out.println(t+" "+maxIndex);
//                System.arraycopy(elementPopCode, 0, stateVals, rowOffset + c*nofFeatureSets, elementPopCode.length);
                
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
        
//        System.out.println("pop");
//        for(t=0; t<stateVals.length; t++){
//            if(t<stateVals.length-1)
//                System.out.print(stateVals[t]+", ");
//        }
//        System.out.println();

        return stateVals;
    }
    
    @Override
    public int [][] sampleStateOfNeurons(double [] par_arrInputDistr, int par_nofSamples){
        // each variable xk is associated
        // with n yi values. n is determined by fanOut value
        int[][] stateSamples = new int[ m_nofNodes ][ par_nofSamples ];
        int nofFeatureSets = m_features.length;
        double [] distr = new double[ nofFeatureSets ];
        int [] samplesSubset_allZero = new int[ nofFeatureSets ];
   
        for(int i=0; i<m_nofNodes; i+=nofFeatureSets){
            
            System.arraycopy(par_arrInputDistr, i, distr, 0, nofFeatureSets);
            
//            if(i==2958){
//                 for(int fi=0; fi<nofFeatureSets; fi++){
//
//                    System.out.print(distr[ fi ] +" ");    
//
//                }                     
//                System.out.println();    
//                
//            }
            int [] samplesSubset;
            RealArray distrV = new RealArray(distr, nofFeatureSets);
            double distrV_sumOfAbs = distrV.uAbs().aSum(); // = 0.0 for non-responsive to any feature
            if( distrV_sumOfAbs > 0.0 ){
            
                samplesSubset = sample(distr, par_nofSamples);
                for(int j=0; j<par_nofSamples; j++){
                
                    stateSamples[ i + samplesSubset[j] ][j] = 1;
                
//                if(i==2958){
//                    System.out.print(samplesSubset[j]+" ");    
//                }
                }   
            }
        }
                //double [] elementPopCode = new double[ nofFeatureSets ];
                //int maxIndex = encode(elementFeatures, elementPopCode);
//                for(int z=0;z<elementFeatures.length;z++){
//                    System.out.print(elementFeatures[z]+" ");
//                }
//                System.out.println();
//                for(int z=0;z<elementPopCode.length;z++){
//                    System.out.print(elementPopCode[z]+" ");
//                }
//                System.out.println();
                
//                // General case: store popCode into stateVal by array copy
//                //i=(a * B * C) + (b * C) + (c * 1)
//                t = nofFeatureSets*(rowOffset + c);
//                //System.out.println(t+" "+maxIndex);
//                System.arraycopy(elementPopCode, 0, stateVals, rowOffset + c*nofFeatureSets, elementPopCode.length);
                
                // Specific case when we have a single 1 for each pop Code, we can perform fewer array access operations
//                t = nofFeatureSets*(rowOffset + c) + maxIndex;
//                // readjust index to account for bias term if any
//                if(m_biasIndex <= t && m_biasIndex >= 0){
//                    t++;
//                }
//                if (elementFeatures[maxIndex] > 0.1);
//                    stateVals[t] += 1;
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
        for(; j<boundIndex; j++){

            //double candidate = Math.abs(par_inputs[j]);
            double candidate = par_inputs[j];
            if(candidate > maxVal){

                maxVal = candidate;
                maxIndex = j;
            }
            
            // don't need to set zero values, can assume they are already set to zero.
            // when created, arrays are automatically initialized with the default value of their type 
            // popCode[ j ] = 0.0;
        }
        popCode[ maxIndex ] = 1.0;
        return maxIndex;
    }
    
    private static double [] evalDistr(double [] par_inputs){
        
        int nofInputs = par_inputs.length;
        double [] inputs = new double[ nofInputs ];
        System.arraycopy(par_inputs, 0, inputs, 0, nofInputs);
        
        RealArray softMax = new RealArray(inputs,nofInputs); 
        softMax = softMax.uExp();      // to handle negative and zero values 
        softMax = softMax.uPow(10);    // to suppress values < 1.0
        //softMax = softMax.uAdd(1.0);   // to handle zero values   
        double sum = softMax.aSum();
        
        softMax = softMax.uMul(1.0/sum);  
            
        return softMax.values();
    }
    
    private static int [] sample(double [] par_distribution, int nofSamples){
              
        int [] samples = new int [ nofSamples ];
        double [] p = ModelUtils.cumulativeSum(par_distribution);
        
//        for(int j=0; j<nofInputs; j++){
//            
//            System.out.print(softMax.values()[j] + " ");
//
//        }
//        System.out.println();
        
        for(int t=0; t<nofSamples; t++){
            
            double r = StdRandom.uniform();
            int j=0;
            while(p[j]<r){
                j++;
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
//            System.out.print(softMax.values()[j] + " ");
//
//        }
//        System.out.println();
        
        double r = StdRandom.uniform();
        int i=0;
        while(p[i]<r){
            i++;
        }

        popCode[ i ] = 1.0;
        return i;
    }
    
    public static void testEncodeViaProb(){
        
        int nofTrials = 100000;
        int nofNodes = 6;
        double[] arrInputs = new double[ nofNodes ];
        
        // set values
        for(int i=0; i<nofNodes; i++){
            
            arrInputs[i] = StdRandom.random();
        }
        
        int [] arrCount = new int [ nofNodes ];
        for(int ti=0; ti<nofTrials; ti++){
            
            double[] popCode = new double [nofNodes];
            int wIndex = encodeViaProb(arrInputs, popCode);
            arrCount[wIndex]++;
        }
        
        for(int i=0; i<nofNodes; i++){

            System.out.print(arrInputs[i] +"  ");
                
         }
        System.out.println();
                
        for(int i=0; i<nofNodes; i++){

            System.out.print(arrCount[i] +"  ");
                
         }
        System.out.println();
    }
    
    public FeaturePopulationCode(){
        
        m_min_response_sum_val = Double.NEGATIVE_INFINITY;
    }
}
