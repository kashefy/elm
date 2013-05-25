/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package model.simulation;

/**
 *
 * @author woodstock
 */
import model.*;
import model.features.*;
import model.encoding.*;
import model.neuron.*;
import model.competition.*;
import model.evaluation.*;
import model.utils.*;
import model.utils.files.*;

import java.io.*;
import model.attention.AbstractAttention;
import model.attention.AttentionSalient;
import model.utils.files.FileIO;

public class SimulationMNIST_layerF extends AbstractSimulation{
    
    // stimuli
    int m_nofRows;
    int m_nofCols;
    int [] m_stimuliDims;
    int [] m_windowDims;
    int m_nofStimElements;
    int m_nofWindowElements;
    int m_totalNofStimuli;
    
    // feature maps
    OrientationMap m_mapOrient;
    
    // encoding
    Encoder m_encoder;
    
    // prediction, learning, classification
    int m_nofYNeurons; // get value from enocder
    int m_nofLearners_layerZ;
    int m_nofLearnersAux;
    int m_nofLearners_layerF;
    ZNeuron [] m_arrZNeurons;
    ZNeuron [] m_arrZNeuronsAux;
    ZNeuron [] m_arrZNeuronsAll;
    ZNeuron [] m_arrZNeurons_layerF;
    
    // competition
    AbstractCompetition m_wta;
    AbstractCompetition m_wtaAll;
    AbstractCompetition m_wta_layerF;
    
    // attention
    int m_nofAttentions;
    AbstractAttention m_attention;
    
    public void setParams(SimulationParams params){
        
        m_params = params;
    }
    
    public void init(){
        
        // stimuli
        m_dataLoader = new DataLoaderImageSetCSV_incremental();
        m_dataLoader.setParams(m_params.getMainInputDir());
        m_dataLoader.init();
        //m_dataLoader.load();
        
        // complete and correct parameters from data loader
        m_totalNofStimuli  = m_dataLoader.getNofSamples();
        m_params.setNofTrainStimuli((int)(m_params.getNofTrainStimuli()/100.0* m_totalNofStimuli));
        m_params.setNofTestStimuli(m_totalNofStimuli-m_params.getNofTrainStimuli());
        m_params.setNofCauses(m_dataLoader.getNofClasses());
        
        m_stimuliDims = m_dataLoader.getDims();
        m_nofStimElements = m_stimuliDims[ FileIO.DIM_INDEX_COLS ] * m_stimuliDims[ FileIO.DIM_INDEX_COLS ];
        
        m_nofAttentions = m_params.getNofAttentions();
        m_attention = new AttentionSalient();
        m_attention.setParams(m_params.getAttentionParamsRef());
        m_attention.init();
        
        m_windowDims = m_attention.getWindowDims();
        m_nofRows     = m_windowDims[ FileIO.DIM_INDEX_COLS ];
        m_params.setNofRows(m_nofRows);
        m_nofCols     = m_windowDims[ FileIO.DIM_INDEX_COLS ];
        m_params.setNofCols(m_nofCols);
        
        m_nofStimElements = m_nofRows * m_nofCols;
        
        // feature maps inits       
        m_mapOrient = new OrientationMap(); 
        m_mapOrient.setStartingScale(0.2*16/m_params.getFeatureMapParamsRef().getSupportRadius());
        m_mapOrient.setParams(m_params.getFeatureMapParamsRef());
        m_mapOrient.init();
        
        EncoderParams encoder_params = new EncoderParams();
        encoder_params.set_feature_map(m_mapOrient);
        encoder_params.set_frequency(m_params.getEncFrequency());
        encoder_params.set_delta_t(m_params.getDeltaT());
        encoder_params.set_duration_milSec(m_params.getEncDurationInMilSec());
        encoder_params.set_nof_pop_code_inputs(m_nofRows*m_nofCols*m_mapOrient.getNofFeatureSets());
        encoder_params.set_pop_code_fan_out(m_params.getPopCodeFanOut());
        encoder_params.set_arr_input_dims(m_windowDims);
        
        // encoding inits
        // need to improve init for m_encoder to conform with other inits
        //m_encoder = new Encoder();
        m_encoder = new EncoderSoftMax();
        m_encoder.set_params(encoder_params);
        m_encoder.init();
        
        // predictors/classifiers
        m_nofYNeurons = m_encoder.getNofEncoderNodes(); // get value from enocder
        
        m_params.getLearnerParams_layerF_Ref().setNofInputNodes(m_nofYNeurons);
        
        m_nofLearners_layerF = m_params.getNofLeaners_layerF();
        m_arrZNeurons_layerF = new ZNeuron[ m_nofLearners_layerF ];       
        
        for(int zi=0; zi<m_nofLearners_layerF; zi++){
            
            //arrZNeurons[li] = new ZNeuron();
            m_arrZNeurons_layerF[zi] = new ZNeuronCompact();
            m_arrZNeurons_layerF[zi].setParams(m_params.getLearnerParams_layerF_Ref());
            m_arrZNeurons_layerF[zi].init();
        } 
        
        m_params.getLearnerParamsRef().setNofInputNodes(m_nofLearners_layerF);
        m_nofLearners_layerZ = m_params.getNofLeaners();
        m_arrZNeurons = new ZNeuron[ m_nofLearners_layerZ ];       
        
        for(int zi=0; zi<m_nofLearners_layerZ; zi++){
            
            //arrZNeurons[li] = new ZNeuron();
            m_arrZNeurons[zi] = new ZNeuronCompact();
            m_arrZNeurons[zi].setParams(m_params.getLearnerParamsRef());
            m_arrZNeurons[zi].init();
        } 
        
        // auxillary learners
        // 0: (predict, fire, update) 100%
        // 1: predict 100%, fire 10%, update 100%
        // 2: predict 100%, fire when no one else fires, update 100%
        // 3: (predict, fire, update) 100% on noise stim only
        m_nofLearnersAux = 4;
        m_arrZNeuronsAux = new ZNeuron[ m_nofLearnersAux ];       
        
        for(int zi=0; zi<m_nofLearnersAux; zi++){
            
            m_arrZNeuronsAux[zi] = new ZNeuronCompact();
            m_arrZNeuronsAux[zi].setParams(m_params.getLearnerParamsRef());
            m_arrZNeuronsAux[zi].init();
        } 
        
        m_arrZNeuronsAll = new ZNeuron[ m_nofLearners_layerZ+m_nofLearnersAux ];
        int relzi = 0;
        for(int zi=0; zi<m_nofLearners_layerZ; zi++, relzi++){
            
            m_arrZNeuronsAll[relzi] = m_arrZNeurons[zi];
        } 
        for(int zi=0; zi<m_nofLearnersAux; zi++, relzi++){
            
            m_arrZNeuronsAll[relzi] = m_arrZNeuronsAux[zi];
        }
        
        // competition inits
        //wta = new CompetitionWTAOU();
        m_wta = new WTAPoissonRate();
        m_wta.setParams(m_params.getCompetitionParamsRef());
        m_wta.init();
        m_wta.refToLearners(m_arrZNeurons);
        
        //m_wtaWithAux = new CompetitionWTAOU();
        m_wtaAll = new WTAPoissonRate();
        m_wtaAll.setParams(m_params.getCompetitionParams_layerF_ref());
        m_wtaAll.init();
        m_wtaAll.refToLearners(m_arrZNeuronsAll);
        
        m_wta_layerF = new WTAPoissonRate();
        m_wta_layerF.setParams(m_params.getCompetitionParams_layerF_ref());
        m_wta_layerF.init();
        m_wta_layerF.refToLearners(m_arrZNeurons_layerF);
        
    }
    
    public void learn(){
        
        // predict - compete - update
        int nofStimuli = m_params.getNofTrainStimuli();
        int gap_width_layerF = m_params.getLearnerParams_layerF_Ref().getHistoryLength();
        int nof_responses_per_window_y2f = m_params.getEncDurationInMilSec() + gap_width_layerF;
        int gap_width_layerZ = m_params.getLearnerParamsRef().getHistoryLength();
        int nof_responses_per_window_f2Z = m_params.getEncDurationInMilSec() + gap_width_layerZ;
        
        //int [][] arrResponse = new int [ nofStimuli ][ nof_responses_per_window_y2f ];
        int [] arr_response1D_layerZ = new int [ nofStimuli * m_nofAttentions * nof_responses_per_window_f2Z ];
        
        //int [][][] arrResponse_layerF = new int [ nofStimuli ][ m_params.getNofAttentions() ][ nof_responses_per_window_y2f ];
        int [] arr_response1D_layerF = new int [ nofStimuli * m_nofAttentions * nof_responses_per_window_y2f ];
        
        // activity mask will be reduced to mask weights of nodes with low activity
        int[] arrTrainLabelNames = m_dataLoader.determineLabelSet(0,nofStimuli-1);
        int nofCauses = m_params.getNofCauses();
        int nofMasks = nofCauses;
//        ActivityMask [] arrActivityMask = new ActivityMask[ nofMasks_layerZ ];
//        for(int mi=0; mi<nofMasks; mi++){
//            
//            arrActivityMask[mi] = new ActivityMask();
//            arrActivityMask[mi].setParams( m_nofRows*m_nofCols );
//            arrActivityMask[mi].set_lower_threshold_excl( m_params.get_activityMaskLowerThresholdExcl() );
//            arrActivityMask[mi].init();
//        }
        
        int [] allZeroSpikeTrain = new int [ m_nofLearners_layerF ];
        int [] allZeroSpikeTrain_layerF = new int [ m_nofYNeurons ];
        
        // let one aux neuron learn noise
        DataLoaderNoise dataLoaderNoise = new DataLoaderNoise();
        dataLoaderNoise.setParams(m_params.getMainInputDir());
        dataLoaderNoise.init();
        dataLoaderNoise.load();
        
        // evaluation
        int predictionStatWindowSize = m_params.get_predictionStatWindowSize();
        
        EvaluationParams evaluation_params = new EvaluationParams();
        evaluation_params.set_nofLearners(m_nofLearners_layerF);
        evaluation_params.set_label_names(arrTrainLabelNames);
        evaluation_params.set_predictionStats_windowSize(predictionStatWindowSize);
                
        PredictionStats predictionStats_perClass_layerF = new PredictionStats();
        predictionStats_perClass_layerF.setParams(evaluation_params);
        predictionStats_perClass_layerF.init();
        
        PredictionStats predictionStats_layerZ = new PredictionStats();
        evaluation_params.set_nofLearners(m_nofLearners_layerZ);
        predictionStats_layerZ.setParams(evaluation_params);
        predictionStats_layerZ.init();
        
        int [] arr_layerF_names = new int [ m_nofLearners_layerF+1 ]; // +1 for no F fires
        for(int fi=0; fi<m_nofLearners_layerF; fi++){
            arr_layerF_names[fi] = fi;
        }
        arr_layerF_names[ m_nofLearners_layerF ] = AbstractCompetition.WTA_NONE;
        
        EvaluationParams evaluation_params_perAttWin = new EvaluationParams();
        evaluation_params_perAttWin.set_nofLearners(1);
        evaluation_params_perAttWin.set_label_names(arr_layerF_names);
        evaluation_params_perAttWin.set_predictionStats_windowSize(nof_responses_per_window_y2f);
        
        PredictionStats predictionStats_perAttWin_layerF = new PredictionStats();
        predictionStats_perAttWin_layerF.setParams(evaluation_params_perAttWin);
        predictionStats_perAttWin_layerF.init();
        
        int nofWeightsToWatch_layerZ = 0;
        double [][][] weightWatch_layerZ = new double [ m_nofLearners_layerZ ][ nofWeightsToWatch_layerZ ][ nofStimuli * m_nofAttentions * nof_responses_per_window_f2Z ];
        //double [][][] rateWatch = new double [ m_nofLearners_layerZ ][ nofWeightsToWatch_layerZ ][  ];
        int [] arrWeightIndicies_layerZ = new int[ nofWeightsToWatch_layerZ ];
        //arrWeightIndicies[0] = 3;
        
        int nofWeightsToWatch_layerF = 0;
        double [][][] weightWatch_layerF = new double [ m_nofLearners_layerF ][ nofWeightsToWatch_layerF ][ nofStimuli * m_nofAttentions * nof_responses_per_window_y2f ];
        //double [][][] rateWatch_layerF = new double [ m_nofLearners_layerF ][ ];
        int [] arrWeightIndicies_layerF = new int[ nofWeightsToWatch_layerF ];
        //arrWeightIndicies_layerF[0] = 50;
        
        int iteration_layerF = 0;
        int iteration_layerZ = 0;
        for(int si=0; si<nofStimuli; si++){
            
            // transform input into set of spike trains
            m_dataLoader.load(); // load here if DataLoaderImageSetCSV_incremental()
            int nCurrent_label = m_dataLoader.getLabel(si);
            if (nCurrent_label == DataLoaderImageSetCSV.INVALID_LABEL){
                
                String message = "Invalid label value " + Integer.toString(nCurrent_label) + "at i=" + Integer.toString(si);
                System.err.println(message);
            }
            double [] stimulus = m_dataLoader.getSample(si);
            m_attention.setScene(stimulus);
            
            for(int ai=0; ai<m_nofAttentions; ai++){
                
                m_attention.attend(null);
                double [] window_of_ttention = m_attention.getWindow();
                
//                for(int li=0; li<nofCauses; li++){
//                
//                    if(nCurrent_label == arrTrainLabelNames[li])
//                        arrActivityMask[li].addSample(window_of_ttention);
//                }
                
                int [][] spikes_y = m_encoder.encode(window_of_ttention);
                int [][] spikes_f = new int [ m_nofLearners_layerF ][ nof_responses_per_window_y2f ]; 

                // for each column in all spike trains of layer F
                // predict - compete - update
                for(int yt=0; yt<nof_responses_per_window_y2f; yt++){

                    // create array for spikes of all y neurons at time yt
                    // traverse through columns of spikes_y[][]
                    int [] spikes_atT_in;
                    int [] spikes_atT_out;
                    if(yt < m_params.getEncDurationInMilSec()){

                        spikes_atT_in = ModelUtils.extractColumns(spikes_y, yt);
                    }
                    else{
                        spikes_atT_in = allZeroSpikeTrain_layerF;
                    }

                    // predict
                    for(int li=0; li<m_nofLearners_layerF; li++){

                        double membranePotential_layerF = m_arrZNeurons_layerF[li].predict(spikes_atT_in);
                        //System.out.println(membranePotential);
                    }

                    // let F's compete before updating
                    int wta_response = m_wta_layerF.compete();
                    
                    if(wta_response != AbstractCompetition.WTA_NONE){

                        int [] arr_wta_response;
                        arr_wta_response = m_wta_layerF.getOutcome();
                        for(int li=0; li<m_nofLearners_layerF; li++){

                            m_arrZNeurons_layerF[ li ].letFire(arr_wta_response[ li ]==1);
                        }
                        spikes_atT_out = arr_wta_response;
                        
                        predictionStats_perClass_layerF.addResponse( wta_response, nCurrent_label );
                        predictionStats_perAttWin_layerF.addResponse( 0, wta_response );
                    }
                    else{
                        spikes_atT_out = allZeroSpikeTrain;
                    }
                    
                    // update F
                    for(int li=0; li<m_nofLearners_layerF; li++){

                        for(int ww=0; ww<nofWeightsToWatch_layerF; ww++){

                            weightWatch_layerF[ li ][ww][ iteration_layerF ] = m_arrZNeurons_layerF[ li ].getWeights()[ arrWeightIndicies_layerF[ww] ];
                        }
                        m_arrZNeurons_layerF[li].update();               
                    }
                    
                    if(yt < nof_responses_per_window_f2Z){
                        ModelUtils.insertColumn(spikes_atT_out, spikes_f, yt);
                    }
                    arr_response1D_layerF[ iteration_layerF ] = wta_response;
                    iteration_layerF++;
                    //System.out.print(wta_response + " ");
                }
                //System.out.println();
                for(int ft=0; ft<nof_responses_per_window_f2Z; ft++){
                    //arrResponse_layerF[ si ][ ai ][ yt ] = wta_response;
                    //arrResponse1D_layerF[ (si*m_nofAttentions+ai)*nof_responses_per_window_y2f+yt ] = wta_response;
                    
                    // create array for f spikes at time ft
                    // traverse through columns of spikes_f[][]
                    int [] spikes_atT;
                    if(ft < nof_responses_per_window_y2f){

                        spikes_atT = ModelUtils.extractColumns(spikes_f, ft);
                    }
                    else{
                        spikes_atT = allZeroSpikeTrain_layerF;
                    }
                    
                    // predict Z
                    for(int li=0; li<m_nofLearners_layerZ; li++){

                        double membranePotential = m_arrZNeurons[li].predict(spikes_atT);
                    }
                    // let Z's compete before updating
                    int wta_response = m_wta.compete();
                    if(wta_response != AbstractCompetition.WTA_NONE){

                        int [] arr_wta_response;
                        arr_wta_response = m_wta.getOutcome();
                        for(int li=0; li<m_nofLearners_layerZ; li++){

                            m_arrZNeurons[ li ].letFire(arr_wta_response[ li ]==1);
                        }
                        predictionStats_layerZ.addResponse( wta_response, nCurrent_label );
                    }

                    // update Z
                    for(int li=0; li<m_nofLearners_layerZ; li++){

                        for(int ww=0; ww<nofWeightsToWatch_layerZ; ww++){
                            
                            weightWatch_layerZ[ li ][ww ][ iteration_layerZ ] = m_arrZNeurons[ li ].getWeights()[ arrWeightIndicies_layerZ[ww] ];
                        }
                        m_arrZNeurons[li].update();               
                    }
                    arr_response1D_layerZ[ iteration_layerZ ] = wta_response;
                    
                    iteration_layerZ++;
                }
            }
        }        
        ModelPredictionTest.saveWeights( new File( m_params.getMainOutputDir(), "weights_layerF.csv" ).getPath(), m_arrZNeurons_layerF);
        ModelPredictionTest.saveBiases( new File( m_params.getMainOutputDir(), "biases_layerF.csv" ).getPath(), m_arrZNeurons_layerF);
        
        ModelPredictionTest.saveWeights( new File( m_params.getMainOutputDir(), "weights.csv" ).getPath(), m_arrZNeurons);
        ModelPredictionTest.saveBiases( new File( m_params.getMainOutputDir(), "biases.csv" ).getPath(), m_arrZNeurons);
        
        FileIO.saveArrayToCSV(arr_response1D_layerF, 1, nofStimuli * m_nofAttentions * nof_responses_per_window_y2f, m_params.getMainOutputDir()+"arr_response1D_layerF.csv");
        FileIO.saveArrayToCSV(arr_response1D_layerZ, 1, nofStimuli * m_nofAttentions * nof_responses_per_window_f2Z, m_params.getMainOutputDir()+"arr_response1D_layerZ.csv");
//        for(int mi=0; mi<nofMasks; mi++){
//            arrActivityMask[mi].calc_activity_intensity();
//        }
//        String strMaskFile = m_params.getMainOutputDir()+"masksClasses.csv";
//        ModelPredictionTest.saveMasks(strMaskFile, arrActivityMask);
        
        File watch_dir = new File(m_params.getMainOutputDir(), "watch");
        
        double [] arrAvgCondEntropy;
        arrAvgCondEntropy = predictionStats_perClass_layerF.get_results_avg_cond_entropy();
        FileIO.saveArrayToCSV( arrAvgCondEntropy, 1, arrAvgCondEntropy.length, new File(watch_dir, "watchAvgCondEntropy_perClass_layerF.csv").getPath() );
        
        arrAvgCondEntropy = predictionStats_perAttWin_layerF.get_results_avg_cond_entropy();
        FileIO.saveArrayToCSV( arrAvgCondEntropy, 1, arrAvgCondEntropy.length, new File(watch_dir, "watchAvgCondEntropy_perAttWin_layerF.csv").getPath() );
            
        arrAvgCondEntropy = predictionStats_layerZ.get_results_avg_cond_entropy();
        FileIO.saveArrayToCSV( arrAvgCondEntropy, 1, arrAvgCondEntropy.length, new File(watch_dir, "watchAvgCondEntropy_layerZ.csv").getPath() );
        
        for(int zi=0; zi<m_nofLearners_layerZ; zi++){
            
            FileIO.saveArrayToCSV(weightWatch_layerZ[zi], new File(watch_dir, "weightWatch"+zi+".csv").getPath());
        }
        
        for(int fi=0; fi<m_nofLearners_layerF; fi++){
            
            FileIO.saveArrayToCSV(weightWatch_layerF[fi], new File(watch_dir, "weightWatch_layerF"+fi+".csv").getPath());
        }
    }
    
    public void intermediate(){
        
        DataLoaderWeightSet weightLoader = new DataLoaderWeightSet();
        weightLoader.setParams(m_params.getMainOutputDir());
        weightLoader.setWeightValueFilename("weights.csv");
        weightLoader.init();
        weightLoader.load();
        DataLoaderWeightSet biasLoader = new DataLoaderWeightSet();
        biasLoader.setParams(m_params.getMainOutputDir());
        biasLoader.setWeightValueFilename("biases.csv");
        biasLoader.init();
        biasLoader.load();
        
        m_arrZNeurons = new ZNeuron[ m_nofLearners_layerZ ];
        
        for(int zi=0; zi<m_nofLearners_layerZ; zi++){
            
            m_arrZNeurons[zi] = new ZNeuronCompact();
            m_arrZNeurons[zi].setParams(m_params.getLearnerParamsRef());
            m_arrZNeurons[zi].init();
            m_arrZNeurons[zi].setWeights(weightLoader.getSample(zi));
            m_arrZNeurons[zi].setBias(biasLoader.getSample(zi)[0]);
        } 
        
        weightLoader = new DataLoaderWeightSet();
        weightLoader.setParams(m_params.getMainOutputDir());
        weightLoader.setWeightValueFilename("weightsAux.csv");
        weightLoader.init();
        weightLoader.load();
        biasLoader = new DataLoaderWeightSet();
        biasLoader.setParams(m_params.getMainOutputDir());
        biasLoader.setWeightValueFilename("biasesAux.csv");
        biasLoader.init();
        biasLoader.load();
        
        for(int zi=0; zi<m_nofLearnersAux; zi++){
            
            m_arrZNeuronsAux[zi] = new ZNeuronCompact();
            m_arrZNeuronsAux[zi].setParams(m_params.getLearnerParamsRef());
            m_arrZNeuronsAux[zi].init();
            m_arrZNeuronsAux[zi].setWeights(weightLoader.getSample(zi));
            m_arrZNeuronsAux[zi].setBias(biasLoader.getSample(zi)[0]);
        } 
    }
    
    public void test(){
        
//        // combine learners with aux learners
//        ZNeuron [] old = m_arrZNeurons;
//        m_arrZNeurons = new ZNeuron[ m_nofLearners_layerZ + m_nofLearnersAux ];
//        for(int i=0; i<m_nofLearners; i++){
//            
//            m_arrZNeurons[i] = old[i];
//        }
//        int reli = m_nofLearners_layerZ;
//        for(int i=0; i<m_nofLearnersAux; i++){
//            
//            m_arrZNeurons[reli++] = m_arrZNeuronsAux[i];
//        }
//        m_nofLearners_layerZ += m_nofLearnersAux;
        
        m_wta.refToLearners(m_arrZNeurons);
        m_wtaAll.refToLearners(m_arrZNeuronsAll);
        
        int starti =  m_params.getNofTrainStimuli();
        int endi = m_totalNofStimuli;
        int nofStimuli = endi-starti+1;
        int gap_width_layerF = m_params.getLearnerParams_layerF_Ref().getHistoryLength();
        int nof_responses_per_window_y2f = m_params.getEncDurationInMilSec()+gap_width_layerF;
        int gap_width_layerZ = m_params.getLearnerParamsRef().getHistoryLength();
        int nof_responses_per_window_f2Z = m_params.getEncDurationInMilSec() + gap_width_layerZ;
        
        int[] arrTestLabelNames = m_dataLoader.determineLabelSet(starti, endi);
        int nofCauses = m_params.getNofCauses();
        
        File dir_activity_layerF = new File(m_params.getMainOutputDir(), "activity_layerF");
        int nofMasks_layerF = m_nofLearners_layerF;
        ActivityMask [] arrActivityMask_layerF = new ActivityMask[ nofMasks_layerF ];
        for(int mi=0; mi<nofMasks_layerF; mi++){
            
            arrActivityMask_layerF[mi] = new ActivityMask();
            arrActivityMask_layerF[mi].setParams(m_nofRows*m_nofCols);
            arrActivityMask_layerF[mi].set_lower_threshold_excl( m_params.get_activityMaskLowerThresholdExcl() );
            arrActivityMask_layerF[mi].enable_logging( dir_activity_layerF.getPath(), Integer.toString(mi) );
            arrActivityMask_layerF[mi].init();
        }
        
        int [] scene_dims = m_attention.getSceneDims();
        int nof_scene_rows = scene_dims[ FileIO.DIM_INDEX_ROWS ];
        int nof_scene_cols = scene_dims[ FileIO.DIM_INDEX_COLS ];
        
        File dir_activity_layerZ = new File(m_params.getMainOutputDir(), "activity_layerZ");
        int nofMasks_layerZ = m_nofLearners_layerZ;
        ActivityMask [] arr_ActivityMask_layerZ = new ActivityMask[ nofMasks_layerZ ];
        for(int mi=0; mi<nofMasks_layerZ; mi++){
            
            arr_ActivityMask_layerZ[mi] = new ActivityMask();
            arr_ActivityMask_layerZ[mi].setParams(nof_scene_rows, nof_scene_cols );
            arr_ActivityMask_layerZ[mi].set_lower_threshold_excl( m_params.get_activityMaskLowerThresholdExcl() );
            arr_ActivityMask_layerZ[mi].enable_logging( dir_activity_layerZ.getPath(), Integer.toString(mi) );
            arr_ActivityMask_layerZ[mi].init();
        }
        
        // membranePotential_layerF : nofZ,nofstimuli,nofAttentios,durationOfSpikeTrain Y for each stimulus per F
        //double [][][][] arr_membranePotential_layerF = new double [ m_nofLearners_layerF ][ nofStimuli ][ m_nofAttentions ][ nof_responses_per_window_y2f ] ;
        int [][][] arrResponse_layerF = new int [ nofStimuli ][ m_nofAttentions ][ nof_responses_per_window_y2f ];
        
        // membranePotentialZ : nofZ,nofstimuli,durationOfSpikeTrain F for each stimulus per Z
        //double [][][] membranePotentialZ = new double [ m_nofLearners_layerZ ][ nofStimuli ][ nof_responses_per_window_y2f ] ;
        int [][] arrResponse_layerZ = new int [ nofStimuli ][ m_nofAttentions*nof_responses_per_window_f2Z ];
        
        int noWinnerCount_layerF = 0;
        int noWinnerCount_layerZ = 0;
        
        PredictionStats predictionStats_layerF = new PredictionStats();
        predictionStats_layerF.setParams(m_nofLearners_layerF, arrTestLabelNames);
        predictionStats_layerF.init();
        
        PredictionStats predictionStats = new PredictionStats();
        predictionStats.setParams(m_nofLearners_layerZ, arrTestLabelNames);
        predictionStats.init();
        
        int [] arr_layerF_names = new int [ m_nofLearners_layerF+1 ]; // +1 for no F fires
        for(int fi=0; fi<m_nofLearners_layerF; fi++){
            arr_layerF_names[fi] = fi;
        }
        arr_layerF_names[ m_nofLearners_layerF ] = AbstractCompetition.WTA_NONE;
        PredictionStats predictionStats_F2Z = new PredictionStats();
        predictionStats_F2Z.setParams(m_nofLearners_layerZ, arr_layerF_names);
        predictionStats_F2Z.init();

        int [] allZeroSpikeTrain = new int [ m_nofLearners_layerF ]; // no F neurons firing
        int [] allZeroSpikeTrain_layerF = new int [ m_nofYNeurons ]; // no Y neurons firing
        
        for(int si=starti, relSi=0; si<endi; si++, relSi++){
            
            m_dataLoader.load(); // load here if DataLoaderImageSetCSV_incremental()
            int nCurrentLabel = m_dataLoader.getLabel(si);
            if (nCurrentLabel == DataLoaderImageSetCSV.INVALID_LABEL){
                
                String message = "Invalid label value " + Integer.toString(nCurrentLabel) + "at i=" + Integer.toString(si);
                System.err.println(message);
            }
            double [] stimulus = m_dataLoader.getSample(si);
            
            m_attention.setScene(stimulus);
            
            for(int ai=0; ai<m_nofAttentions; ai++){
                
                m_attention.attend(null);
                double [] window_of_attention = m_attention.getWindow();
                int [][] spikes_y = m_encoder.encode(window_of_attention);
                int [][] spikes_f = new int [ m_nofLearners_layerF ][ nof_responses_per_window_y2f ];
                
                for(int yt=0; yt<nof_responses_per_window_y2f; yt++){

                    // create array for spikes of all y neurons at time yt
                    // traverse through columns of spikes_y[][]                    
                    int [] spikes_atT_in;
                    int [] spikes_atT_out;
                    if(yt < m_params.getEncDurationInMilSec()){

                        spikes_atT_in = ModelUtils.extractColumns(spikes_y, yt);
                    }
                    else{
                        spikes_atT_in = allZeroSpikeTrain_layerF;
                    }

                    for(int li=0; li<m_nofLearners_layerF; li++){

                        double membranePotential_layerF = m_arrZNeurons_layerF[li].predict(spikes_atT_in);
                        //arr_membranePotential_layerF[fi][ relSi ][ai][ yt ] = membranePotential_layerF;
                    }

                    // let F neurons compete
                    int wta_response_layerF = m_wta_layerF.compete();
                    //System.out.print(wta_response_layerF + " ");
                    if( wta_response_layerF != AbstractCompetition.WTA_NONE ){

                        int [] arr_wta_response_layerF = m_wta_layerF.getOutcome();
                        
                        if( wta_response_layerF == AbstractCompetition.WTA_ALL )
                            wta_response_layerF = PredictionStats.RESPONSE_ALL;
                        predictionStats_layerF.addResponse( wta_response_layerF, nCurrentLabel );
                        arrActivityMask_layerF[ wta_response_layerF ].addSample( window_of_attention );
                        
                        spikes_atT_out = arr_wta_response_layerF;
                    }
                    else{
                        
                        spikes_atT_out = allZeroSpikeTrain;
                        noWinnerCount_layerF++;
                    }
                    ModelUtils.insertColumn(spikes_atT_out, spikes_f, yt);
                    arrResponse_layerF[ relSi ][ai][ yt ] = wta_response_layerF;
                }
                //System.out.println();
                for(int ft=0; ft<nof_responses_per_window_f2Z; ft++){
                    
                    // create array for f spikes at time ft
                    // traverse through columns of spikes_f[][]
                    int [] spikes_atT_in;
                    if(ft < nof_responses_per_window_y2f){

                        spikes_atT_in = ModelUtils.extractColumns(spikes_f, ft);
                        
//                        boolean no_fire_layerF = true;
//                        for(int fi=0; fi<m_nofLearners_layerF; fi++){
//
//                            if(spikes_atT_in[fi] == 1){
//                                no_fire_layerF = false;
//                                System.out.print(fi + " ");
//                            }
//                        }
//                        if(no_fire_layerF){
//                            System.out.print(-1 + " ");
//                        }
                    }
                    else{
                        spikes_atT_in = allZeroSpikeTrain_layerF;
                    }
                        
                    for(int li=0; li<m_nofLearners_layerZ; li++){

                        double membranePotential_layerZ = m_arrZNeurons[li].predict( spikes_atT_in );
                        //membranePotentialZ[zi][ relSi ][ yt ] = membranePotential_layerZ;
                    }
                    
                    // let Z neurons compete
                    int wta_response = m_wta.compete();
                    if(wta_response != AbstractCompetition.WTA_NONE){

                        if(wta_response == AbstractCompetition.WTA_ALL)
                            wta_response = PredictionStats.RESPONSE_ALL;
                        
                        predictionStats.addResponse( wta_response, nCurrentLabel );
                        for(int fi=0; fi<m_nofLearners_layerF; fi++){
                        
                            if(spikes_atT_in[fi] == 1){
                                predictionStats_F2Z.addResponse(wta_response, fi);
                            }
                        }
                        //arr_ActivityMask[ wta_response ].addSample( stimulus );
                        arr_ActivityMask_layerZ[ wta_response ].addSample( window_of_attention, m_attention.getWindowLoc(), m_attention.getWindowDims() );
                    }
                    else{
                        noWinnerCount_layerZ++;
                    }
                    arrResponse_layerZ[ relSi ][ ft ] = wta_response;
                }
                //System.out.println();
            }
        }    

        for(int mi=0; mi<nofMasks_layerF; mi++){
            arrActivityMask_layerF[mi].calc_activity_intensity();
        }
        ModelPredictionTest.saveMasks( new File(m_params.getMainOutputDir(), "masksLearners_layerF.csv").getPath(), arrActivityMask_layerF );
        
        for(int mi=0; mi<nofMasks_layerZ; mi++){
            arr_ActivityMask_layerZ[mi].calc_activity_intensity();
        }
        ModelPredictionTest.saveMasks( new File(m_params.getMainOutputDir(), "masksLearners_layerZ.csv").getPath(), arr_ActivityMask_layerZ );

        //ModelPredictionTest.saveResponses(m_params.getMainOutputDir()+"response_layerF.csv", arrResponse_layerF); 
        ModelPredictionTest.saveResponses(m_params.getMainOutputDir()+"response.csv", arrResponse_layerZ);    
                
        System.out.println("test set size: "+m_params.getNofTestStimuli());
        
        // print prediction stats of layer F
        int [][] firingCounts = predictionStats_layerF.getFiringCounts();
        int [] firingSums = predictionStats_layerF.calcFiringSums();
        double [][] firingProbs = predictionStats_layerF.calcFiringProbs();
        double [] arrCondEntropy = new double [ m_nofLearners_layerF ];
        predictionStats_layerF.calcConditionalEntropy(arrCondEntropy);
        
        System.out.println("layer F:");
        for(int zi=0; zi<m_nofLearners_layerF; zi++){

            for(int ci=0; ci<nofCauses; ci++){

                System.out.print(firingCounts[zi][ci] + "  ");
            }            

            System.out.print("  \u03A3  " + firingSums[zi]);
            System.out.print("  condEntr  " + arrCondEntropy[ zi ]);
            System.out.println();

        }
        System.out.println("No winner: "+noWinnerCount_layerF);
        
        ModelPredictionTest.savePredictionStats(m_params.getMainOutputDir()+"predictionStats_layerF.csv", firingProbs, arrCondEntropy);
        
        // print prediction stats
        firingCounts = predictionStats.getFiringCounts();
        firingSums = predictionStats.calcFiringSums();
        firingProbs = predictionStats.calcFiringProbs();
        arrCondEntropy = new double [ m_nofLearners_layerZ ];
        predictionStats.calcConditionalEntropy(arrCondEntropy);
        
        System.out.println("layer Z:");
        for(int zi=0; zi<m_nofLearners_layerZ; zi++){

            for(int ci=0; ci<nofCauses; ci++){

                System.out.print(firingCounts[zi][ci] + "  ");
            }            

            System.out.print("  \u03A3  " + firingSums[zi]);
            System.out.print("  condEntr  " + arrCondEntropy[ zi ]);
            System.out.println();

        }
        System.out.println("No winner: "+noWinnerCount_layerZ);
        ModelPredictionTest.savePredictionStats(m_params.getMainOutputDir()+"predictionStats.csv", firingProbs, arrCondEntropy);
        
        // print prediction stats F2Z
        firingCounts = predictionStats_F2Z.getFiringCounts();
        firingSums = predictionStats_F2Z.calcFiringSums();
        firingProbs = predictionStats_F2Z.calcFiringProbs();
        arrCondEntropy = new double [ m_nofLearners_layerZ ];
        predictionStats_F2Z.calcConditionalEntropy(arrCondEntropy);
        System.out.println("layer Z:F2Z");
        for(int zi=0; zi<m_nofLearners_layerZ; zi++){

            for(int ci=0; ci<m_nofLearners_layerF+1; ci++){// +1 for no F fires

                System.out.print(firingCounts[zi][ci] + "  ");
            }            
            System.out.print("  \u03A3  " + firingSums[zi]);
            System.out.print("  condEntr  " + arrCondEntropy[ zi ]);
            System.out.println();
        }
        ModelPredictionTest.savePredictionStats(m_params.getMainOutputDir()+"predictionStats_F2Z.csv", firingProbs, arrCondEntropy);
        //m_nofLearners -= m_nofLearnersAux;
    }
    
    public void run(){
    
        System.out.print("learn()...");
        learn();
        System.out.print("done\n");
        //intermediate();
        System.out.print("test()...");
        test();
        System.out.print("done\n");
    }    
 
}

