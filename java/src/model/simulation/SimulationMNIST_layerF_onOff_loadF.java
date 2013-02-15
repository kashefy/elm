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
import java.util.*;
import model.attention.AbstractAttention;
import model.attention.AttentionSalient;
import model.utils.files.FileIO;
import org.shared.array.RealArray;

public class SimulationMNIST_layerF_onOff_loadF extends SimulationMNIST_layerF_onOff{
    
    public void init(){
        
        super.init();
        DataLoaderWeightSet weightLoader = new DataLoaderWeightSet();
        weightLoader.setParams(m_params.getMainOutputDir());
        weightLoader.setWeightValueFilename("weights_layerF.csv");
        weightLoader.init();
        weightLoader.load();
        DataLoaderWeightSet biasLoader = new DataLoaderWeightSet();
        biasLoader.setParams(m_params.getMainOutputDir());
        biasLoader.setWeightValueFilename("biases_layerF.csv");
        biasLoader.init();
        biasLoader.load();
        
        for(int li=0; li<m_nofLearners_layerF; li++){
            
            m_arrZNeurons_layerF[li].setWeights( weightLoader.getSample(li) );
            m_arrZNeurons_layerF[li].setBias( biasLoader.getSample(li)[0] );
        }
        
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
        Arrays.sort(arr_layerF_names);
        
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
                double [] window_of_attention = m_attention.getWindow();
                
//                for(int li=0; li<nofCauses; li++){
//                
//                    if(nCurrent_label == arrTrainLabelNames[li])
//                        arrActivityMask[li].addSample(window_of_ttention);
//                }
                
                int [][] spikes_y_p1 = m_encoder.encode(window_of_attention);
                int [][] spikes_y_p2 = m_encoder_onOff.encode(window_of_attention);
                int [][] spikes_f = new int [ m_nofLearners_layerF ][ nof_responses_per_window_y2f ]; 

                // for each column in all spike trains of layer F
                // predict - compete - update
                for(int yt=0; yt<nof_responses_per_window_y2f; yt++){

                    // create array for spikes of all y neurons at time yt
                    // traverse through columns of spikes_y[][]
                    int [] spikes_atT_in;
                    int [] spikes_atT_out;
                    if(yt < m_params.getEncDurationInMilSec()){

                        int [] spikes_atT_in_p1 = ModelUtils.extractColumns(spikes_y_p1, yt);
                        int [] spikes_atT_in_p2 = ModelUtils.extractColumns(spikes_y_p2, yt);
                        spikes_atT_in = new int [m_nofYNeurons];
                        System.arraycopy(spikes_atT_in_p1, 0, spikes_atT_in, 0, m_encoder.getNofEncoderNodes());
                        System.arraycopy(spikes_atT_in_p2, 0, spikes_atT_in, m_encoder.getNofEncoderNodes(), m_encoder_onOff.getNofEncoderNodes());
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
                    
//                    // update F
//                    for(int li=0; li<m_nofLearners_layerF; li++){
//
//                        for(int ww=0; ww<nofWeightsToWatch_layerF; ww++){
//
//                            weightWatch_layerF[ li ][ww][ iteration_layerF ] = m_arrZNeurons_layerF[ li ].getWeights()[ arrWeightIndicies_layerF[ww] ];
//                        }
//                        m_arrZNeurons_layerF[li].update();               
//                    }
                    
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
        
        FileIO.saveArrayToCSV(arr_response1D_layerF, 1, nofStimuli * m_nofAttentions * nof_responses_per_window_y2f, new File( m_params.getMainOutputDir(), "response1D_layerF_learn.csv").getPath());
        FileIO.saveArrayToCSV(arr_response1D_layerZ, 1, nofStimuli * m_nofAttentions * nof_responses_per_window_f2Z, new File( m_params.getMainOutputDir(), "response1D_layerZ_learn.csv").getPath());
//        for(int mi=0; mi<nofMasks; mi++){
//            arrActivityMask[mi].calc_activity_intensity();
//        }
//        String strMaskFile = m_params.getMainOutputDir()+"masksClasses.csv";
//        ModelPredictionTest.saveMasks(strMaskFile, arrActivityMask);
        
        File watch_dir = new File(m_params.getMainOutputDir(), FileIO.DIR_NAME_WATCH);
        
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
 
}

