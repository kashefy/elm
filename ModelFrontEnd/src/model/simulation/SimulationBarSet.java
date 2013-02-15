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
import model.utils.files.FileIO;

public class SimulationBarSet extends AbstractSimulation{
 
    // stimuli
    int m_nofRows;
    int m_nofCols;
    int [] m_stimuliDims;
    int m_nofStimElements;
    int m_totalNofStimuli;
    
    // feature maps
    OrientationMap m_mapOrient;
    
    // encoding
    Encoder m_encoder;
    
    // prediction, learning, classification
    int m_nofYNeurons; // get value from enocder
    int m_nofLearners;
    ZNeuron [] m_arrZNeurons;
    
    // competition
    AbstractCompetition m_wta;
    
    public void setParams(SimulationParams params){
        
        m_params = params;
 
        /*
        m_strMainOutputDir = params.getMainOutputDir();
        m_strMainInputDir = params.getMainInputDir();

        // data
        m_nofTrainStimuli = params.getNofTrainStimuli();
        m_nofTestStimuli = params.getNofTestStimuli();
        m_nofRows = params.getNofRows();
        m_nofCols = params.getNofCols();

        // encoding parameters
        m_encFrequency = params.getEncFrequency();
        m_deltaT = params.get;
        m_encDurationInMilSec = params.get;
        m_popCodeFanOut = params.get;

        // prediction parameters
        m_nofCauses = params.get;
        m_nofLearners = params.get;

        // parameter objects

        m_learnerParams = params.get;

        m_competitionParams = params.get;

        m_featParams = params.get;
         */
    }
    
    public void init(){
        
        // stimuli
        m_dataLoader = new DataLoaderSimplePatterns();
        m_dataLoader.setParams(m_params.getMainInputDir());
        m_dataLoader.init();
        m_dataLoader.load();
        
        // complete and correct parameters from data loader
        m_totalNofStimuli  = m_dataLoader.getNofSamples();
        m_params.setNofTrainStimuli((int)(m_params.getNofTrainStimuli()/100.0* m_totalNofStimuli));
        m_params.setNofTestStimuli(m_totalNofStimuli-m_params.getNofTrainStimuli());
        m_params.setNofCauses(m_dataLoader.getNofClasses());
        
        m_stimuliDims = m_dataLoader.getDims();
        m_nofRows     = m_stimuliDims[ FileIO.DIM_INDEX_ROWS ];
        m_params.setNofRows(m_nofRows);
        m_nofCols     = m_stimuliDims[ FileIO.DIM_INDEX_COLS ];
        m_params.setNofCols(m_nofCols);
        
        m_nofStimElements = m_nofRows * m_nofCols;
        
        // feature maps inits       
        m_mapOrient = new OrientationMap(); 
        m_mapOrient.setParams(m_params.getFeatureMapParamsRef());
        m_mapOrient.init();
        
        // encoding inits
        // need to improve init for m_encoder to conform with other inits
        //m_encoder = new Encoder();
        m_encoder = new EncoderSoftMax();
        m_encoder.setFeatureMap(m_mapOrient);
        m_encoder.init(m_params.getEncFrequency(), m_params.getDeltaT(), m_params.getEncDurationInMilSec(), 
                m_nofRows*m_nofCols*m_mapOrient.getNofFeatureSets(), m_params.getPopCodeFanOut());
        m_encoder.setInputDimensions(m_stimuliDims);
        
        // predictors/classifiers
        m_nofYNeurons = m_encoder.getNofEncoderNodes(); // get value from enocder
        m_params.getLearnerParamsRef().setNofInputNodes(m_nofYNeurons);
        m_nofLearners = m_params.getNofLeaners();
        m_arrZNeurons = new ZNeuron[ m_nofLearners ];       
        
        for(int zi=0; zi<m_nofLearners; zi++){
            
            //arrZNeurons[zi] = new ZNeuron();
            m_arrZNeurons[zi] = new ZNeuronCompact();
            m_arrZNeurons[zi].setParams(m_params.getLearnerParamsRef());
            m_arrZNeurons[zi].init();
        } 
        
        // competition inits
        //wta = new CompetitionWTAOU();
        m_wta = new CompetitionWTAPoissonRate();
        m_wta.setParams(m_params.getCompetitionParamsRef());
        m_wta.init();
        m_wta.refToLearners(m_arrZNeurons);
        
    }
    
    public void learn(){
        
        // predict - compete - update
        int nofStimuli = m_params.getNofTrainStimuli();
        int gapWidth = m_params.getLearnerParamsRef().getHistoryLength();
        int nofResponsesPerStimulus = m_params.getEncDurationInMilSec()+gapWidth;
        int [][] arrResponse = new int [ nofStimuli ][ nofResponsesPerStimulus ];
        int [] arrResponse1D = new int [ nofStimuli * nofResponsesPerStimulus];

        // activity mask will be used to mask weights of nodes with low activity
        int[] arrTrainLabelNames = determineLabelSet(0,nofStimuli-1);
        int nofCauses = m_params.getNofCauses();
        int nofMasks = nofCauses;
        double activityMaskLowerThresholdExcl = 0.05;
        ActivityMask [] arrActivityMask = new ActivityMask[ nofMasks ];
        for(int mi=0; mi<nofMasks; mi++){
            
            arrActivityMask[mi] = new ActivityMask();
            arrActivityMask[mi].setParams(m_nofRows*m_nofCols);
            arrActivityMask[mi].set_lower_threshold_excl(activityMaskLowerThresholdExcl);
            arrActivityMask[mi].init();
        }
        
        int [] allZeroSpikeTrain = new int [ m_nofYNeurons ];
        
        int relSi = 0;
        for(int si=0; si<nofStimuli; si++){
            
            int responseIndex = 0;
                
            // transform input into set of spike trains
            int nCurrentLabel = m_dataLoader.getLabel(si);
            double [] stimulus = m_dataLoader.getSample(si);
            int [][] spikesY = m_encoder.encode(stimulus);
            for(int li=0; li<nofCauses; li++){
                
                if(nCurrentLabel == arrTrainLabelNames[li])
                    arrActivityMask[li].addSample(stimulus);
            }   

            // for each column in all spike trains
            // predict - compete - update
            for(int yt=0; yt<nofResponsesPerStimulus; yt++){

                // create array for spikes of all y neurons at time yt
                // traverse through columns of spikesY[][]
                int [] spikesYAtT;
                if(yt < m_params.getEncDurationInMilSec()){

                    spikesYAtT = ModelUtils.extractColumns(spikesY, yt);
                }
                else
                    spikesYAtT = allZeroSpikeTrain;

                // predict
                for(int zi=0; zi<m_nofLearners; zi++){

                    double membranePotential;
                    membranePotential = m_arrZNeurons[zi].predict(spikesYAtT);
                }

                // let Z's compete before updating
                int wtaResponse = m_wta.compete();
                //System.out.println(wtaResponse);

                if(wtaResponse != AbstractCompetition.WTA_NONE){

                    int [] arrWTAResponse;
                    arrWTAResponse = m_wta.getOutcome();
                    for(int zi=0; zi<m_nofLearners; zi++){

                        m_arrZNeurons[ zi ].letFire(arrWTAResponse[ zi ]==1);
                        //    m_arrZNeurons[ zi ].letFire(false);
                    }
                }

                // update/learn
                for(int zi=0; zi<m_nofLearners; zi++){

                    m_arrZNeurons[zi].update();               
                }
                
                arrResponse[ relSi ][ responseIndex++ ] = wtaResponse;
                arrResponse1D[ relSi*nofResponsesPerStimulus+yt ] = wtaResponse;
                
            }   
            relSi++;
        }        

        for(int mi=0; mi<nofMasks; mi++){
            arrActivityMask[mi].calc_activity_intensity();
        }
        String strMaskFile = m_params.getMainOutputDir()+"masksClasses.csv";
        ModelPredictionTest.saveMasks(strMaskFile, arrActivityMask);
        
        int predictionStatWindowSize = 20; // in no. of stimuli
        int nofPredictionStatSamples =  nofStimuli/predictionStatWindowSize;
        double [] arrAvgCondEntropy = new double[ nofPredictionStatSamples ];

        PredictionStats predictionStats = new PredictionStats();
        predictionStats.setParams(m_nofLearners,arrTrainLabelNames);
        predictionStats.init();
            
        int pi = 0;
        for(int si=0; si<nofStimuli-predictionStatWindowSize; si+=predictionStatWindowSize){
                     
            for(int subsi=0; subsi<predictionStatWindowSize; subsi++){
                
                int label = m_dataLoader.getLabel(si+subsi);
                for(int ri=0; ri<nofResponsesPerStimulus; ri++){
                    
                     int response = arrResponse[si+subsi][ri];
                     if(response != AbstractCompetition.WTA_NONE){
                        predictionStats.addResponse(response, label);
                     }
                }
               
            }
            arrAvgCondEntropy[pi] = predictionStats.calcConditionalEntropy();
            predictionStats.reset();
            
            pi++;
            
        }
        FileIO.saveArrayToCSV(arrAvgCondEntropy,1,nofPredictionStatSamples,m_params.getMainOutputDir()+"avgCondEntropy.csv");
    }
    
    public void test(){
        
        int starti =  m_params.getNofTrainStimuli();
        int endi = m_totalNofStimuli;
        int nofStimuli = endi-starti+1;
        int gapWidth = m_params.getLearnerParamsRef().getHistoryLength();
        int nofResponsesPerStimulus = m_params.getEncDurationInMilSec()+gapWidth;
        
        int[] arrTestLabelNames = determineLabelSet(starti,endi);
        int nofCauses = m_params.getNofCauses();
        
        int nofMasks = m_nofLearners;
        double activityMaskLowerThresholdExcl = 0.05;
        ActivityMask [] arrActivityMask = new ActivityMask[ nofMasks ];
        for(int mi=0; mi<nofMasks; mi++){
            
            arrActivityMask[mi] = new ActivityMask();
            arrActivityMask[mi].setParams(m_nofRows*m_nofCols);
            arrActivityMask[mi].set_lower_threshold_excl(activityMaskLowerThresholdExcl);
            arrActivityMask[mi].init();
        }
        
        // membranePotentialZ : nofZ,nofstimuli,durationOfSpikeTrain Y for each stimulus per Z
        double [][][] membranePotentialZ = new double [ m_nofLearners ][ nofStimuli ][ nofResponsesPerStimulus ] ;
        int [][] arrResponse = new int [ nofStimuli ][ nofResponsesPerStimulus ];
        int noWinnerCount = 0;
        
        PredictionStats predictionStats = new PredictionStats();
        predictionStats.setParams(m_nofLearners,arrTestLabelNames);
        predictionStats.init();

        int [] allZeroSpikeTrain = new int [ m_nofYNeurons ];
        
        int relSi = 0;
        for(int si=starti; si<endi; si++){
                
            int nCurrentLabel = m_dataLoader.getLabel(si);
            double [] stimulus = m_dataLoader.getSample(si);
            int [][] spikesY = m_encoder.encode(stimulus);
            //int [][] temp = ModelUtils.pad2D(spikesY, 6, ModelUtils.MODE_PAD_COLS);

            for(int yt=0; yt<nofResponsesPerStimulus; yt++){

                // create array for spikes of all y neurons at time yt
                // traverse through columns of spikesY[][]                    
                int [] spikesYAtT = (yt < m_params.getEncDurationInMilSec())? ModelUtils.extractColumns(spikesY, yt) : allZeroSpikeTrain;

                for(int zi=0; zi<m_nofLearners; zi++){

                    membranePotentialZ[zi][ relSi ][ yt ] = m_arrZNeurons[zi].predict(spikesYAtT);
                }

                int wtaResponse = m_wta.compete();
                
                if(wtaResponse != AbstractCompetition.WTA_NONE){
                    
                    if(wtaResponse == AbstractCompetition.WTA_ALL)
                        wtaResponse = PredictionStats.RESPONSE_ALL;
                    predictionStats.addResponse(wtaResponse, nCurrentLabel);
                    
                    arrActivityMask[wtaResponse].addSample(stimulus);
                }
                else 
                    noWinnerCount++;

                arrResponse[ relSi ][ yt ] = wtaResponse;
                
            }
            relSi++;
        }   
        for(int mi=0; mi<nofMasks; mi++){
            arrActivityMask[mi].calc_activity_intensity();
        }
        ModelPredictionTest.saveMasks(m_params.getMainOutputDir()+"masksLearners.csv", arrActivityMask);

        ModelPredictionTest.saveResponses(m_params.getMainOutputDir()+"response.csv", arrResponse);
        ModelPredictionTest.saveWeights(m_params.getMainOutputDir()+"weights.csv", m_arrZNeurons);    
                
        System.out.println("test set size: "+m_params.getNofTestStimuli());
        
        // print prediction stats
        int [][] firingCounts = predictionStats.getFiringCounts();
        int [] firingSums = predictionStats.calcFiringSums();
        double [][] firingProbs = predictionStats.calcFiringProbs();
        double [] arrCondEntropy = new double [ m_nofLearners ];
        predictionStats.calcConditionalEntropy(arrCondEntropy);
        
        for(int zi=0; zi<m_nofLearners; zi++){

            for(int ci=0; ci<nofCauses; ci++){

                System.out.print(firingCounts[zi][ci] + "  ");
            }            

            System.out.print("  \u03A3  " + firingSums[zi]);
            System.out.print("  condEntr  " + arrCondEntropy[ zi ]);
            System.out.println();

        }
        System.out.println("No winner: "+noWinnerCount);
        
        ModelPredictionTest.savePredictionStats(m_params.getMainOutputDir()+"predictionStats.csv", firingProbs, arrCondEntropy);
  
    }
    
    public void run(){
    
        System.out.println("running");
        learn();
        System.out.println("learn()...done");
        test();
        System.out.println("test()...done");
    } 
}
