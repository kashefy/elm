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
import org.shared.array.RealArray;

public class Simulation_MNIST_layerIndep extends AbstractSimulation{
               
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
        
        // encoding inits
        // need to improve init for m_encoder to conform with other inits
        //m_encoder = new Encoder();
        m_encoder = new EncoderSoftMax();
        m_encoder.setFeatureMap(m_mapOrient);
        m_encoder.init(m_params.getEncFrequency(), m_params.getDeltaT(), m_params.getEncDurationInMilSec(), 
                m_nofRows*m_nofCols*m_mapOrient.getNofFeatureSets(), m_params.getPopCodeFanOut());
        m_encoder.setInputDimensions(m_windowDims);
        
        // predictors/classifiers
        m_nofYNeurons = m_encoder.getNofEncoderNodes(); // get value from enocder
        
        m_params.getLearnerParams_layerF_Ref().setNofInputNodes(m_nofYNeurons);
        
        m_nofLearners_layerF = m_params.getNofLeaners_layerF();
        m_arrZNeurons_layerF = new ZNeuron[ m_nofLearners_layerF ];       
        
        for(int fi=0; fi<m_nofLearners_layerF; fi++){
            
            //arrZNeurons[fi] = new ZNeuron();
            m_arrZNeurons_layerF[fi] = new ZNeuronCompact();
            m_arrZNeurons_layerF[fi].setParams(m_params.getLearnerParams_layerF_Ref());
            m_arrZNeurons_layerF[fi].init();
        } 
        
        m_params.getLearnerParamsRef().setNofInputNodes(m_nofLearners_layerF);
        m_nofLearners_layerZ = m_params.getNofLeaners();
        m_arrZNeurons = new ZNeuron[ m_nofLearners_layerZ ];       
        
        for(int zi=0; zi<m_nofLearners_layerZ; zi++){
            
            //arrZNeurons[fi] = new ZNeuron();
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
        m_wta_layerF.setParams(m_params.getCompetitionParamsRef());
        m_wta_layerF.init();
        m_wta_layerF.refToLearners(m_arrZNeurons_layerF);
        
    }
    
    public void learn(){
        
        // predict - compete - update
        int nofStimuli = m_params.getNofTrainStimuli();
        int gapWidth_layerF = m_params.getLearnerParams_layerF_Ref().getHistoryLength();
        int gapWidth_layerZ = m_params.getLearnerParamsRef().getHistoryLength();
        int nofResponsesPerWindow_Y2F = m_params.getEncDurationInMilSec() + gapWidth_layerF;
        int nofResponsesPerWindow_F2Z = m_params.getEncDurationInMilSec() + gapWidth_layerZ;
        
        //int [][] arrResponse = new int [ nofStimuli ][ nofResponsesPerWindow ];
        //int [] arrResponse1D = new int [ nofStimuli * nofResponsesPerWindow ];
        
        //int [][][] arrResponse_layerF = new int [ nofStimuli ][ m_params.getNofAttentions() ][ nofResponsesPerWindow ];
        //int [] arrResponse1D_layerF = new int [ nofStimuli * m_params.getNofAttentions() * nofResponsesPerWindow ];
        
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
        
        int [] allZeroSpikeTrain_Y2F = new int [ m_nofYNeurons ];
        int [] allZeroSpikeTrain_F2Z = new int [ m_nofLearners_layerF ];
        
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
        
        PredictionStats predictionStats_layerF = new PredictionStats();
        predictionStats_layerF.setParams(evaluation_params);
        predictionStats_layerF.init();
        
        PredictionStats predictionStats = new PredictionStats();
        evaluation_params.set_nofLearners(m_nofLearners_layerZ);
        predictionStats.setParams(evaluation_params);
        predictionStats.init();
        
        int nofWeightsToWatch_layerF = 1;
        double [][][] weightWatch_layerF = new double [ m_nofLearners_layerF ][ nofWeightsToWatch_layerF ][ nofStimuli * m_nofAttentions * nofResponsesPerWindow_Y2F ];
        //double [][][] rateWatch_layerF = new double [ m_nofLearners_layerF ][ ];
        int [] arrWeightIndicies_layerF = new int[ nofWeightsToWatch_layerF ];
        arrWeightIndicies_layerF[0] = 50;
        
        int nofWeightsToWatch_layerZ = 1;
        double [][][] weightWatch_layerZ = new double [ m_nofLearners_layerZ ][ nofWeightsToWatch_layerZ ][ nofStimuli * m_nofAttentions * nofResponsesPerWindow_F2Z ];
        //double [][][] rateWatch = new double [ m_nofLearners_layerZ ][ nofWeightsToWatch ][  ];
        int [] arrWeightIndicies_layerZ = new int[ nofWeightsToWatch_layerZ ];
        arrWeightIndicies_layerZ[0] = 3;
        
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
                double [] windowOfAttention = m_attention.getWindow();
                
//                for(int li=0; li<nofCauses; li++){
//                
//                    if(nCurrent_label == arrTrainLabelNames[li])
//                        arrActivityMask[li].add_sample(windowOfAttention);
//                }
                
                int [][] spikesY = m_encoder.encode(windowOfAttention); // [ Z ][ ty ]
                int [][] spikesF_actual = new int[ nofResponsesPerWindow_Y2F ][ m_nofLearners_layerF ]; // transposed format of spikesY
                int [][] spikesF_resampled = new int[ nofResponsesPerWindow_F2Z ][ m_nofLearners_layerF ]; // transposed format of spikesY
                Histogram spiking_hist_layerF = new Histogram();
                spiking_hist_layerF.setParams(m_nofLearners_layerF);
                spiking_hist_layerF.init();

                // for each column in all spike trains of layer F
                // predict - compete - update
                for(int ty=0; ty<nofResponsesPerWindow_Y2F; ty++){

                    // create array for spikes of all y neurons at time ty
                    // traverse through columns of spikesY[][]
                    int [] spikesAtT;
                    if(ty < m_params.getEncDurationInMilSec()){

                        spikesAtT = ModelUtils.extract_columns(spikesY, ty);
                    }
                    else{
                        spikesAtT = allZeroSpikeTrain_Y2F;
                    }

                    // predict
                    for(int fi=0; fi<m_nofLearners_layerF; fi++){

                        double membranePotential_layerF = m_arrZNeurons_layerF[fi].predict(spikesAtT);
                    }

                    // let F's compete before updating
                    int wtaResponse_layerF = m_wta_layerF.compete();
                    int [] arr_WTAResponse_layerF = null;
                    
                    if(wtaResponse_layerF != AbstractCompetition.WTA_NONE){

                        arr_WTAResponse_layerF = m_wta_layerF.getOutcome();
                        for(int fi=0; fi<m_nofLearners_layerF; fi++){

                            m_arrZNeurons_layerF[ fi ].letFire(arr_WTAResponse_layerF[ fi ]==1);
                        }
                        spikesF_actual[ty] = arr_WTAResponse_layerF;
                        spiking_hist_layerF.increment(wtaResponse_layerF);
                        
                        predictionStats_layerF.addResponse( wtaResponse_layerF, nCurrent_label );
                    }
                    
                    // update F
                    for(int fi=0; fi<m_nofLearners_layerF; fi++){

                        for(int ww=0; ww<nofWeightsToWatch_layerF; ww++){

                            weightWatch_layerF[ fi ][ww][ iteration_layerF ] = m_arrZNeurons_layerF[ fi ].getWeights()[ arrWeightIndicies_layerF[ww] ];
                        }
                        m_arrZNeurons_layerF[fi].update();               
                    }
                    //arrResponse_layerF[ si ][ ai ][ ty ] = wtaResponse_layerF;
                    //arrResponse1D_layerF[ (si*m_nofAttentions+ai)*nofResponsesPerWindow+ty ] = wtaResponse_layerF;
                    iteration_layerF++;
                }
                
                // resample layer F spike train
                Distribution1D spiking_distr_layerF = new Distribution1D();
                spiking_distr_layerF.setParams(m_nofLearners_layerF);
                spiking_distr_layerF.init();
                spiking_distr_layerF.evalDistr(spiking_hist_layerF.normalize());
                int nof_samples_per_to_merge = m_nofLearners_layerF/3;
                nof_samples_per_to_merge = (nof_samples_per_to_merge > 0)? nof_samples_per_to_merge : 1; // produce at least 1 sample
                for(int tf=0; tf<nofResponsesPerWindow_F2Z; tf++){
                    
                    int [] samples = spiking_distr_layerF.sample(nof_samples_per_to_merge);
                    for(int tfi=0; tfi<nof_samples_per_to_merge; tfi++){
                        
                        spikesF_resampled[tf][samples[tfi]] = 1;
                    }
                }
                int [][] spikesF = spikesF_resampled;
                
                for(int tf=0; tf<nofResponsesPerWindow_F2Z; tf++){
                    
                    int [] spikesAtT;
                    if(tf < m_params.getEncDurationInMilSec()){

                        spikesAtT = spikesF[tf];
                    }
                    else{
                        spikesAtT = allZeroSpikeTrain_F2Z;
                    }
                    // predict Z
                    for(int zi=0; zi<m_nofLearners_layerZ; zi++){

                        double membranePotential = m_arrZNeurons[zi].predict(spikesAtT);
                    }
                    // let Z's compete before updating
                    int wtaResponse = m_wta.compete();
                    if(wtaResponse != AbstractCompetition.WTA_NONE){

                        int [] arrWTAResponse;
                        arrWTAResponse = m_wta.getOutcome();
                        for(int zi=0; zi<m_nofLearners_layerZ; zi++){

                            m_arrZNeurons[ zi ].letFire(arrWTAResponse[ zi ]==1);
                        }
                        predictionStats.addResponse( wtaResponse, nCurrent_label );
                    }

                    // update Z
                    for(int zi=0; zi<m_nofLearners_layerZ; zi++){

                        for(int ww=0; ww<nofWeightsToWatch_layerZ; ww++){
                            
                            weightWatch_layerZ[ zi ][ww ][ iteration_layerZ ] = m_arrZNeurons[ zi ].getWeights()[ arrWeightIndicies_layerZ[ww] ];
                        }
                        m_arrZNeurons[zi].update();               
                    }
                }
            }
        }
        ModelPredictionTest.saveWeights( new File( m_params.getMainOutputDir(), "weights_layerF.csv" ).getPath(), m_arrZNeurons_layerF);
        ModelPredictionTest.saveBiases( new File( m_params.getMainOutputDir(), "biases_layerF.csv" ).getPath(), m_arrZNeurons_layerF);
        
        ModelPredictionTest.saveWeights( new File( m_params.getMainOutputDir(), "weights.csv" ).getPath(), m_arrZNeurons);
        ModelPredictionTest.saveBiases( new File( m_params.getMainOutputDir(), "biases.csv" ).getPath(), m_arrZNeurons);
        
        //FileIO.saveArrayToCSV(arrResponse1D_layerF, 1, nofStimuli * m_params.getNofAttentions() * nofResponsesPerWindow, m_params.getMainOutputDir()+"arrResponse1D_layerF.csv");

//        for(int mi=0; mi<nofMasks; mi++){
//            arrActivityMask[mi].calc_activity_intensity();
//        }
//        String strMaskFile = m_params.getMainOutputDir()+"masksClasses.csv";
//        ModelPredictionTest.saveMasks(strMaskFile, arrActivityMask);
        
        File watch_dir = new File(m_params.getMainOutputDir(), "watch");
        
        double [] arrAvgCondEntropy;
        arrAvgCondEntropy = predictionStats_layerF.get_results_avg_cond_entropy();
        FileIO.saveArrayToCSV( arrAvgCondEntropy, 1, arrAvgCondEntropy.length, new File(watch_dir, "watchAvgCondEntropy_layerF.csv").getPath() );
            
        arrAvgCondEntropy = predictionStats_layerF.get_results_avg_cond_entropy();
        FileIO.saveArrayToCSV( arrAvgCondEntropy, 1, arrAvgCondEntropy.length, new File(watch_dir, "watchAvgCondEntropy.csv").getPath() );
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
        int gapWidth_layerF = m_params.getLearnerParams_layerF_Ref().getHistoryLength();
        int gapWidth_layerZ = m_params.getLearnerParamsRef().getHistoryLength();
        int nofResponsesPerWindow_Y2F = m_params.getEncDurationInMilSec() + gapWidth_layerF;
        int nofResponsesPerWindow_F2Z = m_params.getEncDurationInMilSec() + gapWidth_layerZ;
        
        int[] arrTestLabelNames = m_dataLoader.determineLabelSet(starti, endi);
        int nofCauses = m_params.getNofCauses();
        
        int nofMasks_layerF = m_nofLearners_layerF;
        ActivityMask [] arrActivityMask_layerF = new ActivityMask[ nofMasks_layerF ];
        for(int mi=0; mi<nofMasks_layerF; mi++){
            
            arrActivityMask_layerF[mi] = new ActivityMask();
            arrActivityMask_layerF[mi].setParams( m_attention.getWindowDims()[ FileIO.DIM_INDEX_ROWS ]*
                    m_attention.getWindowDims()[ FileIO.DIM_INDEX_COLS ] );
            arrActivityMask_layerF[mi].set_lower_threshold_excl( m_params.get_activityMaskLowerThresholdExcl() );
            arrActivityMask_layerF[mi].enable_logging( new File(m_params.getMainOutputDir(), "activity_layerF").getPath(), Integer.toString(mi) );
            arrActivityMask_layerF[mi].init();
        }
        
        int nofMasks_layerZ = m_nofLearners_layerZ;
        ActivityMask [] arr_ActivityMask = new ActivityMask[ nofMasks_layerZ ];
        for(int mi=0; mi<nofMasks_layerZ; mi++){
            
            arr_ActivityMask[mi] = new ActivityMask();
            arr_ActivityMask[mi].setParams( m_nofRows*m_nofCols );
            arr_ActivityMask[mi].set_lower_threshold_excl( m_params.get_activityMaskLowerThresholdExcl() );
            arr_ActivityMask[mi].enable_logging( new File(m_params.getMainOutputDir(), "activity").getPath(), Integer.toString(mi) );
            arr_ActivityMask[mi].init();
        }
        
        // membranePotential_layerF : nofZ,nofstimuli,nofAttentios,durationOfSpikeTrain Y for each stimulus per F
        double [][][][] membranePotential_layerF = new double [ m_nofLearners_layerF ][ nofStimuli ][ m_params.getNofAttentions() ][ nofResponsesPerWindow_Y2F ] ;
        int [][][] arrResponse_layerF = new int [ nofStimuli ][ m_params.getNofAttentions() ][ nofResponsesPerWindow_Y2F ];
        
        // membranePotentialZ : nofZ,nofstimuli,durationOfSpikeTrain F for each stimulus per Z
        double [][][] membranePotentialZ = new double [ m_nofLearners_layerZ ][ nofStimuli ][ nofResponsesPerWindow_F2Z ] ;
        int [][] arrResponse = new int [ nofStimuli ][ nofResponsesPerWindow_F2Z ];
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

        int [] allZeroSpikeTrain_Y2F = new int [ m_nofYNeurons ]; // no Y neurons firing
        int [] allZeroSpikeTrain_F2Z = new int [ m_nofLearners_layerF ]; // no F neurons firing
        
        for(int si=starti, relSi=0; si<endi; si++, relSi++){
            
            m_dataLoader.load(); // load here if DataLoaderImageSetCSV_incremental()
            int nCurrentLabel = m_dataLoader.getLabel(si);
            if (nCurrentLabel == DataLoaderImageSetCSV.INVALID_LABEL){
                
                String message = "Invalid label value " + Integer.toString(nCurrentLabel) + "at i=" + Integer.toString(si);
                System.err.println(message);
            }
            double [] stimulus = m_dataLoader.getSample(si);
            
            m_attention.setScene(stimulus);
            
            for(int ai=0; ai<m_params.getNofAttentions(); ai++){
                
                m_attention.attend(null);
                double [] windowOfAttention = m_attention.getWindow();
                int [][] spikesY = m_encoder.encode(windowOfAttention);
                int [][] spikesF_actual = new int[ nofResponsesPerWindow_Y2F ][ m_nofLearners_layerF ]; // transposed format of spikesY
                int [][] spikesF_resampled = new int[ nofResponsesPerWindow_F2Z ][ m_nofLearners_layerF ]; // transposed format of spikesY
                Histogram spiking_hist_layerF = new Histogram();
                spiking_hist_layerF.setParams(m_nofLearners_layerF);
                spiking_hist_layerF.init();

                for(int ty=0; ty<nofResponsesPerWindow_Y2F; ty++){

                    // create array for spikes of all y neurons at time ty
                    // traverse through columns of spikesY[][]                    
                    int [] spikesAtT;
                    if(ty < m_params.getEncDurationInMilSec()){

                        spikesAtT = ModelUtils.extract_columns(spikesY, ty);
                    }
                    else{
                        spikesAtT = allZeroSpikeTrain_Y2F;
                    }

                    for(int fi=0; fi<m_nofLearners_layerF; fi++){

                        membranePotential_layerF[fi][ relSi ][ai][ ty ] = m_arrZNeurons_layerF[fi].predict(spikesAtT);
                    }

                    // let F neurons compete
                    int wta_response_layerF = m_wta_layerF.compete();
                    if( wta_response_layerF != AbstractCompetition.WTA_NONE ){

                        int [] arr_wta_response_layerF = m_wta_layerF.getOutcome();
                        
                        if( wta_response_layerF == AbstractCompetition.WTA_ALL )
                            wta_response_layerF = PredictionStats.RESPONSE_ALL;
                        
                        spikesF_actual[ty] = arr_wta_response_layerF;
                        spiking_hist_layerF.increment(wta_response_layerF);
                        
                        predictionStats_layerF.addResponse( wta_response_layerF, nCurrentLabel );
                        arrActivityMask_layerF[ wta_response_layerF ].add_sample( windowOfAttention );//////////////
                    }
                    else{
                        noWinnerCount_layerF++;
                    }
                    
                    arrResponse_layerF[ relSi ][ai][ ty ] = wta_response_layerF;
                }
                
                // resample layer F spike train
                Distribution1D spiking_distr_layerF = new Distribution1D();
                spiking_distr_layerF.setParams(m_nofLearners_layerF);
                spiking_distr_layerF.init();
                spiking_distr_layerF.evalDistr(spiking_hist_layerF.normalize());
                int nof_samples_per_t = m_nofLearners_layerF/3;
                nof_samples_per_t = (nof_samples_per_t > 0)? nof_samples_per_t : 1; // produce at least 1 sample
                for(int tf=0; tf<nofResponsesPerWindow_F2Z; tf++){
                    
                    int [] samples = spiking_distr_layerF.sample(nof_samples_per_t);
                    for(int tfi=0; tfi<nof_samples_per_t; tfi++){
                        
                        spikesF_resampled[tf][samples[tfi]] = 1;
                    }
                }
                int [][] spikesF = spikesF_resampled;
                
                for(int tf=0; tf<nofResponsesPerWindow_F2Z; tf++){
                    
                    int [] spikesAtT;
                    if(tf < m_params.getEncDurationInMilSec()){

                        spikesAtT = spikesF[tf];
                    }
                    else{
                        spikesAtT = allZeroSpikeTrain_F2Z;
                    }
                    
                    for(int zi=0; zi<m_nofLearners_layerZ; zi++){

                        membranePotentialZ[zi][ relSi ][ tf ] = m_arrZNeurons[zi].predict( spikesAtT );
                    }
                    
                    // let Z neurons compete
                    int wta_response = m_wta.compete();
                    if(wta_response != AbstractCompetition.WTA_NONE){

                        if(wta_response == AbstractCompetition.WTA_ALL)
                            wta_response = PredictionStats.RESPONSE_ALL;
                        predictionStats.addResponse( wta_response, nCurrentLabel );
                        
                        for(int fi=0; fi<spikesAtT.length; fi++){
                        
                            if(spikesAtT[fi] > 0)
                                predictionStats_F2Z.addResponse(wta_response, fi);
                        }
                        arr_ActivityMask[ wta_response ].add_sample( stimulus );
                    }
                    else{
                        noWinnerCount_layerZ++;
                    }
                    arrResponse[ relSi ][ tf ] = wta_response;
                }
            }
        }    

        for(int mi=0; mi<nofMasks_layerF; mi++){
            arrActivityMask_layerF[mi].calc_activity_intensity();
        }
        ModelPredictionTest.saveMasks( m_params.getMainOutputDir()+"masksLearners_layerF.csv", arrActivityMask_layerF );
        
        for(int mi=0; mi<nofMasks_layerZ; mi++){
            arr_ActivityMask[mi].calc_activity_intensity();
        }
        ModelPredictionTest.saveMasks( m_params.getMainOutputDir()+"masksLearners.csv", arr_ActivityMask );

        //ModelPredictionTest.saveResponses(m_params.getMainOutputDir()+"response_layerF.csv", arrResponse_layerF); 
        ModelPredictionTest.saveResponses(m_params.getMainOutputDir()+"response.csv", arrResponse);    
                
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


