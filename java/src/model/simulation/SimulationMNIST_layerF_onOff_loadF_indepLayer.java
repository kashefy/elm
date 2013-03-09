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

public class SimulationMNIST_layerF_onOff_loadF_indepLayer extends SimulationMNIST_layerF_onOff{
    
    private boolean m_bload_layer_f = true;
    
    public void init(){
        
        super.init();
        if(m_bload_layer_f){
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

            for(int li=0; li<m_nofLearners_layerF; ++li){

                m_arrZNeurons_layerF[li].setWeights( weightLoader.getSample(li) );
                m_arrZNeurons_layerF[li].setBias( biasLoader.getSample(li)[0] );
            }
        }
    }
    
    public void learn(){
        
        // predict - compete - update
        int nofStimuli = m_params.getNofTrainStimuli();
        int gap_width_layerF = m_params.getLearnerParams_layerF_Ref().getHistoryLength();
        int nof_responses_per_window_y2f = m_params.getEncDurationInMilSec() + gap_width_layerF;
        int nof_responses_per_stimulus_y2f = m_nofAttentions*nof_responses_per_window_y2f;
        int gap_width_layerZ = m_params.getLearnerParamsRef().getHistoryLength();
        int nof_responses_per_stimulus_f2Z = nof_responses_per_stimulus_y2f + gap_width_layerZ;
        
        //int [][] arrResponse = new int [ nofStimuli ][ nof_responses_per_window_y2f ];
        int [] arr_response1D_layerZ = new int [ nofStimuli * nof_responses_per_stimulus_f2Z ];
        
        //int [][][] arrResponse_layerF = new int [ nofStimuli ][ m_params.getNofAttentions() ][ nof_responses_per_window_y2f ];
        int [] arr_response1D_layerF = new int [ nofStimuli * nof_responses_per_stimulus_y2f ];
        
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
        double [][][] weightWatch_layerZ = new double [ m_nofLearners_layerZ ][ nofWeightsToWatch_layerZ ][ nofStimuli * nof_responses_per_stimulus_f2Z ];
        //double [][][] rateWatch = new double [ m_nofLearners_layerZ ][ nofWeightsToWatch_layerZ ][  ];
        int [] arrWeightIndicies_layerZ = new int[ nofWeightsToWatch_layerZ ];
        //arrWeightIndicies[0] = 3;
        
        int nofWeightsToWatch_layerF = 0;
        double [][][] weightWatch_layerF = new double [ m_nofLearners_layerF ][ nofWeightsToWatch_layerF ][ nofStimuli * nof_responses_per_stimulus_y2f ];
        //double [][][] rateWatch_layerF = new double [ m_nofLearners_layerF ][ ];
        int [] arrWeightIndicies_layerF = new int[ nofWeightsToWatch_layerF ];
        //arrWeightIndicies_layerF[0] = 50;
        
        int iteration_layerF = 0;
        int iteration_layerZ = 0;
        for(int si=0; si<nofStimuli; ++si){
            
            // transform input into set of spike trains
            m_dataLoader.load(); // load here if DataLoaderImageSetCSV_incremental()
            int nCurrent_label = m_dataLoader.getLabel(si);
            if (nCurrent_label == DataLoaderImageSetCSV.INVALID_LABEL){
                
                String message = "Invalid label value " + Integer.toString(nCurrent_label) + "at i=" + Integer.toString(si);
                System.err.println(message);
            }
            double [] stimulus = m_dataLoader.getSample(si);
            m_attention.setScene(stimulus);
            
            int [][] spikes_f_per_stimulus = new int [ m_nofLearners_layerF ][ nof_responses_per_stimulus_y2f ];
            
            for(int ai=0; ai<m_nofAttentions; ++ai){
                
                int attention_offset = ai*nof_responses_per_window_y2f;
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
                for(int yt=0; yt<nof_responses_per_window_y2f; ++yt){

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

                    // predict F
                    for(int li=0; li<m_nofLearners_layerF; ++li){

                        double membranePotential_layerF = m_arrZNeurons_layerF[li].predict(spikes_atT_in);
                        //System.out.println(membranePotential);
                    }

                    // let F's compete before updating
                    int wta_response = m_wta_layerF.compete();
                    
                    if(wta_response != AbstractCompetition.WTA_NONE){

                        int [] arr_wta_response;
                        arr_wta_response = m_wta_layerF.getOutcome();
                        for(int li=0; li<m_nofLearners_layerF; ++li){

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
                    for(int li=0; li<m_nofLearners_layerF; ++li){

                        for(int ww=0; ww<nofWeightsToWatch_layerF; ++ww){

                            weightWatch_layerF[ li ][ww][ iteration_layerF ] = m_arrZNeurons_layerF[ li ].getWeights()[ arrWeightIndicies_layerF[ww] ];
                        }
                        if(!m_bload_layer_f){
                            m_arrZNeurons_layerF[li].update();
                        }
                    }
                    
                    ModelUtils.insertColumn(spikes_atT_out, spikes_f, yt);
                    ModelUtils.insertColumn(spikes_atT_out, spikes_f_per_stimulus, attention_offset+yt);
                    arr_response1D_layerF[ iteration_layerF ] = wta_response;
                    iteration_layerF++;
                    //System.out.print(wta_response + " ");
                }
            }
            
            int [][] spikes_f2z;
            spikes_f2z = resample_spike_train_layerF(spikes_f_per_stimulus);
            spikes_f2z = refactor_spike_train_layerF(spikes_f2z);
            
            for(int ft=0; ft<nof_responses_per_stimulus_y2f; ++ft){
                //arrResponse_layerF[ si ][ ai ][ yt ] = wta_response;
                //arrResponse1D_layerF[ (si*m_nofAttentions+ai)*nof_responses_per_window_y2f+yt ] = wta_response;

                // create array for f spikes at time ft
                // traverse through columns of spikes_f[][]
                int [] spikes_atT;
                if(ft < nof_responses_per_stimulus_y2f){

                    spikes_atT = ModelUtils.extractColumns(spikes_f2z, ft);
                }
                else{
                    spikes_atT = allZeroSpikeTrain_layerF;
                }

                // predict Z
                for(int li=0; li<m_nofLearners_layerZ; ++li){

                    double membranePotential = m_arrZNeurons[li].predict(spikes_atT);
                }
                // let Z's compete before updating
                int wta_response = m_wta.compete();
                if(wta_response != AbstractCompetition.WTA_NONE){

                    int [] arr_wta_response;
                    arr_wta_response = m_wta.getOutcome();
                    for(int li=0; li<m_nofLearners_layerZ; ++li){

                        m_arrZNeurons[ li ].letFire(arr_wta_response[ li ]==1);
                    }
                    predictionStats_layerZ.addResponse( wta_response, nCurrent_label );
                }

                // update Z
                for(int li=0; li<m_nofLearners_layerZ; ++li){

                    for(int ww=0; ww<nofWeightsToWatch_layerZ; ++ww){

                        weightWatch_layerZ[ li ][ww ][ iteration_layerZ ] = m_arrZNeurons[ li ].getWeights()[ arrWeightIndicies_layerZ[ww] ];
                    }
                    m_arrZNeurons[li].update();               
                }
                arr_response1D_layerZ[ iteration_layerZ ] = wta_response;

                ++iteration_layerZ;
            }
        }        
        ModelPredictionTest.saveWeights( new File( m_params.getMainOutputDir(), "weights_layerF.csv" ).getPath(), m_arrZNeurons_layerF);
        ModelPredictionTest.saveBiases( new File( m_params.getMainOutputDir(), "biases_layerF.csv" ).getPath(), m_arrZNeurons_layerF);
        
        ModelPredictionTest.saveWeights( new File( m_params.getMainOutputDir(), "weights.csv" ).getPath(), m_arrZNeurons);
        ModelPredictionTest.saveBiases( new File( m_params.getMainOutputDir(), "biases.csv" ).getPath(), m_arrZNeurons);
        
        FileIO.saveArrayToCSV(arr_response1D_layerF, 1, nofStimuli * m_nofAttentions * nof_responses_per_window_y2f, new File( m_params.getMainOutputDir(), "response1D_layerF_learn.csv").getPath());
        FileIO.saveArrayToCSV(arr_response1D_layerZ, 1, nofStimuli * nof_responses_per_stimulus_f2Z, new File( m_params.getMainOutputDir(), "response1D_layerZ_learn.csv").getPath());
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
    public void test(){
        
        m_wta.refToLearners(m_arrZNeurons);
        m_wtaAll.refToLearners(m_arrZNeuronsAll);
        
        int starti =  m_params.getNofTrainStimuli();
        int endi = m_totalNofStimuli;
        int nofStimuli = endi-starti+1;
        int gap_width_layerF = m_params.getLearnerParams_layerF_Ref().getHistoryLength();
        int nof_responses_per_window_y2f = m_params.getEncDurationInMilSec()+gap_width_layerF;
        int nof_responses_per_stimulus_y2f = m_nofAttentions*nof_responses_per_window_y2f;
        int gap_width_layerZ = m_params.getLearnerParamsRef().getHistoryLength();
        int nof_responses_per_stimulus_f2Z = nof_responses_per_stimulus_y2f + gap_width_layerZ;
        
        int[] arrTestLabelNames = m_dataLoader.determineLabelSet(starti, endi);
        int nofCauses = m_params.getNofCauses();
        
        File dir_activity_layerF = new File(m_params.getMainOutputDir(), FileIO.DIR_NAME_ACTIVITY_LAYER_F);
        int nofMasks_layerF = m_nofLearners_layerF;
        ActivityMask [] arrActivityMask_layerF = new ActivityMask[ nofMasks_layerF ];
        for(int mi=0; mi<nofMasks_layerF; ++mi){
            
            arrActivityMask_layerF[mi] = new ActivityMask();
            arrActivityMask_layerF[mi].setParams(m_nofRows*m_nofCols);
            arrActivityMask_layerF[mi].set_lower_threshold_excl( m_params.get_activityMaskLowerThresholdExcl() );
            arrActivityMask_layerF[mi].enable_logging( dir_activity_layerF.getPath(), Integer.toString(mi) );
            arrActivityMask_layerF[mi].init();
        }
        
        int [] scene_dims = m_attention.getSceneDims();
        int nof_scene_rows = scene_dims[ FileIO.DIM_INDEX_ROWS ];
        int nof_scene_cols = scene_dims[ FileIO.DIM_INDEX_COLS ];
        
        File dir_activity_layerZ = new File(m_params.getMainOutputDir(), FileIO.DIR_NAME_ACTIVITY_LAYER_Z);
        int nofMasks_layerZ = m_nofLearners_layerZ;
        ActivityMask [] arr_ActivityMask_layerZ = new ActivityMask[ nofMasks_layerZ ];
        for(int mi=0; mi<nofMasks_layerZ; ++mi){
            
            arr_ActivityMask_layerZ[mi] = new ActivityMask();
            arr_ActivityMask_layerZ[mi].setParams(nof_scene_rows, nof_scene_cols );
            arr_ActivityMask_layerZ[mi].set_lower_threshold_excl( m_params.get_activityMaskLowerThresholdExcl() );
            arr_ActivityMask_layerZ[mi].enable_logging( dir_activity_layerZ.getPath(), Integer.toString(mi) );
            arr_ActivityMask_layerZ[mi].init();
        }
        
        // membranePotential_layerF : nofZ,nofstimuli,nofAttentios,durationOfSpikeTrain Y for each stimulus per F
        //double [][][][] arr_membranePotential_layerF = new double [ m_nofLearners_layerF ][ nofStimuli ][ m_nofAttentions ][ nof_responses_per_window_y2f ] ;
        int [][][] arrResponse_layerF = new int [ nofStimuli ][ m_nofAttentions ][ nof_responses_per_window_y2f ];
        
        // membrane_pot_layerZ : nofZ,nofstimuli,durationOfSpikeTrain F for each stimulus per Z
        double [][] membrane_pot_layerZ = new double [ m_nofLearners_layerZ ][ nofStimuli*nof_responses_per_stimulus_f2Z ] ;
        int [][] arrResponse_layerZ = new int [ nofStimuli ][ nof_responses_per_stimulus_f2Z ];
        
        int noWinnerCount_layerF = 0;
        int noWinnerCount_layerZ = 0;
        
        PredictionStats predictionStats_layerF = new PredictionStats();
        predictionStats_layerF.setParams(m_nofLearners_layerF, arrTestLabelNames);
        predictionStats_layerF.init();
        
        PredictionStats predictionStats = new PredictionStats();
        predictionStats.setParams(m_nofLearners_layerZ, arrTestLabelNames);
        predictionStats.init();
        
        int [] arr_layerF_names = new int [ m_nofLearners_layerF+1 ]; // +1 for no F fires
        for(int fi=0; fi<m_nofLearners_layerF; ++fi){
            arr_layerF_names[fi] = fi;
        }
        arr_layerF_names[ m_nofLearners_layerF ] = AbstractCompetition.WTA_NONE;
        Arrays.sort(arr_layerF_names);
        
        PredictionStats predictionStats_F2Z = new PredictionStats();
        predictionStats_F2Z.setParams(m_nofLearners_layerZ, arr_layerF_names);
        predictionStats_F2Z.init();

        int [] allZeroSpikeTrain = new int [ m_nofLearners_layerF ]; // no F neurons firing
        int [] allZeroSpikeTrain_layerF = new int [ m_nofYNeurons ]; // no Y neurons firing
        
        for(int si=starti, relSi=0; si<endi; si++, ++relSi){
            
            m_dataLoader.load(); // load here if DataLoaderImageSetCSV_incremental()
            int nCurrentLabel = m_dataLoader.getLabel(si);
            if (nCurrentLabel == DataLoaderImageSetCSV.INVALID_LABEL){
                
                String message = "Invalid label value " + Integer.toString(nCurrentLabel) + "at i=" + Integer.toString(si);
                System.err.println(message);
            }
            double [] stimulus = m_dataLoader.getSample(si);
            m_attention.setScene(stimulus);
            
            int [][] spikes_f_per_stimulus = new int [ m_nofLearners_layerF ][ nof_responses_per_stimulus_y2f ];
            
            double [][] arr_window_of_attention = new double[ m_nofAttentions ][ m_nofRows*m_nofCols ];
            
            for(int ai=0; ai<m_nofAttentions; ++ai){
                
                int attention_offset = ai*nof_responses_per_window_y2f;
                m_attention.attend(null);
                double [] window_of_attention = m_attention.getWindow();
                System.arraycopy(window_of_attention, 0, arr_window_of_attention[ai], 0, m_nofRows*m_nofCols);
                int [][] spikes_y_p1 = m_encoder.encode(window_of_attention);
                int [][] spikes_y_p2 = m_encoder_onOff.encode(window_of_attention);
                int [][] spikes_f = new int [ m_nofLearners_layerF ][ nof_responses_per_window_y2f ];
                
                for(int yt=0; yt<nof_responses_per_window_y2f; ++yt){

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

                    for(int li=0; li<m_nofLearners_layerF; ++li){

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
                    ModelUtils.insertColumn(spikes_atT_out, spikes_f_per_stimulus, attention_offset+yt);
                    arrResponse_layerF[ relSi ][ai][ yt ] = wta_response_layerF;
                }
            }

            int [][] spikes_f2z;
            spikes_f2z = resample_spike_train_layerF(spikes_f_per_stimulus);
            spikes_f2z = refactor_spike_train_layerF(spikes_f2z);
            
            for(int ft=0; ft<nof_responses_per_stimulus_f2Z; ++ft){

                // create array for f spikes at time ft
                // traverse through columns of spikes_f[][]
                int [] spikes_atT_in;
                if(ft < nof_responses_per_window_y2f){

                    spikes_atT_in = ModelUtils.extractColumns(spikes_f2z, ft);

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

                for(int li=0; li<m_nofLearners_layerZ; ++li){

                    double membranePotential_layerZ = m_arrZNeurons[li].predict( spikes_atT_in );
                    membrane_pot_layerZ[li][ relSi*nof_responses_per_stimulus_f2Z+ft ] = membranePotential_layerZ;
                }

                // let Z neurons compete
                int wta_response = m_wta.compete();
                if(wta_response != AbstractCompetition.WTA_NONE){

                    if(wta_response == AbstractCompetition.WTA_ALL)
                        wta_response = PredictionStats.RESPONSE_ALL;

                    predictionStats.addResponse( wta_response, nCurrentLabel );
                    boolean layerF_fired = false;
                    for(int fi=0; fi<m_nofLearners_layerF; ++fi){

                        if(spikes_atT_in[fi] == 1){
                            predictionStats_F2Z.addResponse(wta_response, fi);
                            layerF_fired = true;
                        }
                    }
                    if( !layerF_fired ){
                        predictionStats_F2Z.addResponse(wta_response, AbstractCompetition.WTA_NONE);
                    }
                    //arr_ActivityMask[ wta_response ].addSample( stimulus );
                    //arr_ActivityMask_layerZ[ wta_response ].addSample( window_of_attention, m_attention.getWindowLoc(), m_attention.getWindowDims() );
                }
                else{
                    noWinnerCount_layerZ++;
                }
                arrResponse_layerZ[ relSi ][ ft ] = wta_response;
            }
            //System.out.println();
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
        ModelPredictionTest.saveResponses(new File(m_params.getMainOutputDir(),"response_layerZ_test.csv").getPath(), arrResponse_layerZ);    
        FileIO.saveArrayToCSV(membrane_pot_layerZ, new File(m_params.getMainOutputDir(),"membrane_pot_layerZ.csv").getPath());
        
        System.out.println("test set size: "+m_params.getNofTestStimuli());
        
        // print prediction stats of layer F
        int [][] firingCounts = predictionStats_layerF.getFiringCounts();
        int [] arr_firingSums = predictionStats_layerF.calcFiringSums();
        double [][] arr_firing_probs = predictionStats_layerF.calcFiringProbs();
        double [] arr_firingSums_temp = new double[m_nofLearners_layerF];
        double [] arrCondEntropy = new double [ m_nofLearners_layerF ];
        predictionStats_layerF.calcConditionalEntropy(arrCondEntropy);
        
        System.out.println("layer F:");
        for(int li=0; li<m_nofLearners_layerF; ++li){

            for(int ci=0; ci<nofCauses; ci++){

                System.out.print(firingCounts[li][ci] + "  ");
            }            

            System.out.print("  \u03A3  " + arr_firingSums[li]);
            System.out.print("  condEntr  " + arrCondEntropy[ li ]);
            System.out.println();
            arr_firingSums_temp[li] = (double)arr_firingSums[li];
        }
        System.out.println("No winner count: "+noWinnerCount_layerF);
        // average firing sum during testing:
        RealArray firing_sums = new RealArray(arr_firingSums_temp);
        System.out.format("Mean firing sum= %.0f%n", firing_sums.aMean());
        m_log_results.format("%.0f,", firing_sums.aMean());
        
        // uniformity of firing
        double firing_uniformity = firing_sums.uMul(1.0/firing_sums.aSum()).aEnt();
        System.out.format("Uniformity of firing probs= %.3f%n", firing_uniformity);
        m_log_results.format("%.3f,", firing_uniformity);

        ModelPredictionTest.savePredictionStats(new File(m_params.getMainOutputDir(), "predictionStats_layerF.csv").getPath(), arr_firing_probs, arrCondEntropy);
        
        // print prediction stats
        firingCounts = predictionStats.getFiringCounts();
        arr_firingSums = predictionStats.calcFiringSums();
        arr_firing_probs = predictionStats.calcFiringProbs();
        arr_firingSums_temp = new double[m_nofLearners_layerZ];
        arrCondEntropy = new double [ m_nofLearners_layerZ ];
        predictionStats.calcConditionalEntropy(arrCondEntropy);
        
        System.out.println("layer Z:");
        for(int li=0; li<m_nofLearners_layerZ; ++li){

            for(int ci=0; ci<nofCauses; ci++){

                System.out.print(firingCounts[li][ci] + "  ");
            }            

            System.out.print("  \u03A3  " + arr_firingSums[li]);
            System.out.print("  condEntr  " + arrCondEntropy[ li ]);
            System.out.println();
            arr_firingSums_temp[li] = (double)arr_firingSums[li];
        }
        System.out.println("No winner: "+noWinnerCount_layerZ);
        // average firing sum during testing:
        firing_sums = new RealArray(arr_firingSums_temp);
        System.out.format("Mean firing sum= %.0f%n", firing_sums.aMean());
        m_log_results.format("%.0f,", firing_sums.aMean());
        
        // uniformity of firing
        firing_uniformity = firing_sums.uMul(1.0/firing_sums.aSum()).aEnt();
        System.out.format("Uniformity of firing probs= %.3f%n", firing_uniformity);
        m_log_results.format("%.3f,", firing_uniformity);
        
        ModelPredictionTest.savePredictionStats(new File(m_params.getMainOutputDir(), "predictionStats.csv").getPath(), arr_firing_probs, arrCondEntropy);
        
        // print prediction stats F2Z
        firingCounts = predictionStats_F2Z.getFiringCounts();
        arr_firingSums = predictionStats_F2Z.calcFiringSums();
        arr_firing_probs = predictionStats_F2Z.calcFiringProbs();
        arr_firingSums_temp = new double[ m_nofLearners_layerZ ];
        arrCondEntropy = new double [ m_nofLearners_layerZ ];
        predictionStats_F2Z.calcConditionalEntropy(arrCondEntropy);
        System.out.println("layer Z:F2Z");
        for(int li=0; li<m_nofLearners_layerZ; ++li){

            for(int ci=0; ci<m_nofLearners_layerF+1; ++ci){// +1 for no F fires

                System.out.print(firingCounts[li][ci] + "  ");
            }            
            System.out.print("  \u03A3  " + arr_firingSums[li]);
            System.out.print("  condEntr  " + arrCondEntropy[ li ]);
            System.out.println();
            arr_firingSums_temp[li] = (double)arr_firingSums[li];
        }
        // average firing sum during testing:
        firing_sums = new RealArray(arr_firingSums_temp);
        System.out.format("Mean firing sum= %.0f.%n", firing_sums.aMean());
        m_log_results.format("%.0f,", firing_sums.aMean());
        
        // uniformity of firing
        firing_uniformity = firing_sums.uMul(1.0/firing_sums.aSum()).aEnt();
        System.out.format("Uniformity of firing probs= %.3f%n", firing_uniformity);
        m_log_results.format("%.3f,", firing_uniformity);
        
        ModelPredictionTest.savePredictionStats(new File(m_params.getMainOutputDir(), "predictionStats_F2Z.csv").getPath(), arr_firing_probs, arrCondEntropy);
        //m_nofLearners -= m_nofLearnersAux;
        
        m_log_results.println();
        m_log_results.close();
    }
    
    // resample spikes from layer F
    private int [][] resample_spike_train_layerF(int[][] par_spike_train){
        
        int n = par_spike_train[0].length;
        int [][] result = new int[m_nofLearners_layerF][n];
        
        Histogram spiking_hist_layerF = new Histogram();
        spiking_hist_layerF.setParams(m_nofLearners_layerF+1); // +1 when no F fires
        spiking_hist_layerF.init();
        for(int ft=0; ft<n; ++ft){

            int[] spikes_at_T = ModelUtils.extractColumns(par_spike_train, ft);
            boolean bno_f_fired = true;
            for(int i=0; i<m_nofLearners_layerF; ++i){

                if(spikes_at_T[i]> 0){
                    spiking_hist_layerF.increment(i);
                    bno_f_fired = false;
                }
            }
            if(bno_f_fired){

                spiking_hist_layerF.increment(m_nofLearners_layerF);
            }
        }
        Distribution1D spiking_distr_layerF = new Distribution1D();
        spiking_distr_layerF.setParams(m_nofLearners_layerF);
        spiking_distr_layerF.init();
        spiking_distr_layerF.evalDistr(spiking_hist_layerF.normalize());

        int nof_samples_per_to_merge = 1; // produce at least 1 sample
        for(int ft=0; ft<n; ++ft){

            int [] samples = spiking_distr_layerF.sample(nof_samples_per_to_merge);
            for(int sample_i=0; sample_i<nof_samples_per_to_merge; sample_i++){

                int ft_spiking_node = samples[sample_i];
                if(ft_spiking_node<m_nofLearners_layerF){
                    result[ft_spiking_node][ft] = 1;
                }
            }
        }

        // verify resampled spike train has similar distribution
//
//        Histogram spiking_hist_layerF2 = new Histogram();
//        spiking_hist_layerF2.setParams(m_nofLearners_layerF+1); // +1 when no F fires
//        spiking_hist_layerF2.init();
//        for(int ft=0; ft<n; ++ft){
//
//            int[] spikes_at_T = ModelUtils.extractColumns(result, ft);
//            boolean bno_f_fired = true;
//            for(int i=0; i<m_nofLearners_layerF; ++i){
//
//                if(spikes_at_T[i]> 0){
//                    spiking_hist_layerF2.increment(i);
//                    bno_f_fired = false;
//                }
//            }
//            if(bno_f_fired){
//
//                spiking_hist_layerF2.increment(m_nofLearners_layerF);
//            }
//
//        }
//        double[] x = spiking_hist_layerF.normalize();
//        double[] y = spiking_hist_layerF2.normalize();
//        double z = 0;
//        for(int i=0; i<x.length; ++i){
//            z += Math.abs(x[i]-y[i]);
//        }
//        System.out.println(z/x.length);
//        System.out.println(java.util.Arrays.toString(x));
//        System.out.println(java.util.Arrays.toString(y));
        return result;
    }
    
    // re-invent layer F spike train
    private int [][] refactor_spike_train_layerF(int[][] par_spike_train){
        
        int n = par_spike_train[0].length;
        
        // bi state (m_nofLearners_layerF*2)
        SimplePopulationCode bi_state_pop_code = new SimplePopulationCode();
        bi_state_pop_code.init(m_nofLearners_layerF, 2);
        
        // ranking per f (m_nofLearners_layerF*m_nofLearners_layerF)
        
        int [][] result = new int[ m_nofLearners_layerF*2 + m_nofLearners_layerF*m_nofLearners_layerF][n];
        for(int ft=0; ft<n; ++ft){

            int[] spikes_at_T = ModelUtils.extractColumns(par_spike_train, ft);
            ModelUtils.insertColumn(bi_state_pop_code.calcStateOfNeurons(spikes_at_T), result, ft);
        }
        
        return result;
    }
}

