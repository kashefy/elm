/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package model.encoding;

/**
 *
 * @author woodstock
 */

import model.features.OrientationMapTest;
import model.features.OrientationMap;
import model.*;
import org.shared.array.*;

public class EncoderTest {
    
    // test simple population code
    public void test1(){
        
        int nofRows     = 2;
        int nofCols     = 1;
        double frequency    = 40;       // 40 Hz
        double deltaT       = 0.001;    // 1 ms
        int duration        = 500;
        int popCodeFanOut = 3;
        
        Encoder encoder;
        encoder = new Encoder();
        
        encoder.init(frequency, deltaT, duration,nofRows*nofCols,popCodeFanOut);
        
        
        double [][] stimuli = new double [3][nofRows*nofCols];
        stimuli[0] = new double[]{1.0,0.0,0.0,0.0,0.0,0.0};
        stimuli[1] = new double[]{0.0,0.0,0.0,0.0,0.0,0.0};
        stimuli[2] = OrientationMapTest.genBarStimulusVals(nofRows,nofCols,nofCols/2,nofRows/2,1,90);
        
        int nofTimeSteps = 1;
        int nofNodes = encoder.getNofEncoderNodes();
        int [][] spikes = new int[nofNodes][duration];
//        for(int step=0; step<nofTimeSteps; step++){
//            
//            spikes = encoder.encode(stimuli[step]);
//        }
        
        int nodesToPrint = 4;
        
        System.out.println("pop code");

        double [] p = encoder.genPopCode(stimuli[0]);
        for(int yi=0; yi<nodesToPrint; yi++){

            System.out.print(p[yi]+", ");
            
        }
        System.out.println();
        
        p = encoder.genPopCode(stimuli[1]);       
        for(int yi=0; yi<nodesToPrint; yi++){

            System.out.print(p[yi]+", ");
            
        }
        System.out.println();
        
        
        
        spikes = encoder.encode(stimuli[0]);
        System.out.println("spike trains");
        for(int yi=0; yi<nodesToPrint; yi++){
            
            System.out.print("n"+yi+": ");
            for(int t=0; t<duration; t++){

                System.out.print(spikes[yi][t]);
            } 
            System.out.println();
        }
       
        System.out.println("stimuli[1]");
        spikes = encoder.encode(stimuli[1]);
        
        for(int yi=0; yi<nodesToPrint; yi++){
            
            System.out.print("n"+yi+": ");
            for(int t=0; t<duration; t++){

                System.out.print(spikes[yi][t]);
            } 
            System.out.println();
        }
       
        // get firing rate of neuron choose durationInMilSec = 1000
        double rate = 0;
        for(int i=0; i<duration; i++){
            
            rate += spikes[3][i];
        }
        rate /= (double)duration;
        System.out.print("rate "+rate);       
    }
    
    // test feature population code
    public void test2(){
        
        // encoder parameters
        Encoder encoder = new Encoder();
        int nofRows     = 3;
        int nofCols     = 3;
        double frequencyEncoding = 40;  // 40 Hz
        double deltaT       = 0.001;    // 1 ms
        int durationInMilSec        = 50;       // ms
        int popCodeFanOut = 1;

        // orientation map parameters
        OrientationMap mapOrient = new OrientationMap(); 
        int supportRadius = 1;
        int nofScales = 1;
        double scalingFactor = 1.5;
        double orientationResolution = 30.0;
        double gaborFrequency = 1.0;
        double elongation = 0.65;

        // stimuli parameters
        int [] stimuliDims = new int[]{nofRows,nofCols};
        int nofStimuli = 6;
        double [][] stimuli = new double [ nofStimuli ][ nofRows*nofCols ];
 
        // combined initializations
        mapOrient.init(supportRadius,nofScales,scalingFactor,orientationResolution,gaborFrequency,elongation);
        
        encoder.setFeatureMap(mapOrient);
        encoder.init(frequencyEncoding, deltaT, durationInMilSec, nofRows*nofCols*mapOrient.getNofFeatureSets(), popCodeFanOut);
        encoder.setInputDimensions(stimuliDims);
        
        stimuli[0] = OrientationMapTest.genBarStimulusVals(nofRows,nofCols,nofCols/2,nofRows/2,1,90);
        stimuli[1] = OrientationMapTest.genBarStimulusVals(nofRows,nofCols,nofCols/2,nofRows/2,1,0);
        stimuli[2] = OrientationMapTest.genBarStimulusVals(nofRows,nofCols,nofCols/2,nofRows/2,1,30);
        stimuli[3] = OrientationMapTest.genBarStimulusVals(nofRows,nofCols,nofCols/2,nofRows/2,1,60);
        stimuli[4] = OrientationMapTest.genBarStimulusVals(nofRows,nofCols,nofCols/2,nofRows/2,1,120);
        stimuli[5] = OrientationMapTest.genBarStimulusVals(nofRows,nofCols,nofCols/2,nofRows/2,1,150);
        //stimuli[2] = OrientationMapTest.genBarStimulusVals(nofRows,nofCols,nofCols/2,nofRows/2,1,45);       


        int nofTimeSteps = 1;
        int nofNodes = encoder.getNofEncoderNodes();
        int [][] spikes = new int[ nofNodes ][ durationInMilSec ];
//        for(int step=0; step<nofTimeSteps; step++){
//            
//            spikes = encoder.encode(stimuli[step]);
//        }

        System.out.println("pop code");
        
        double [] p = encoder.genPopCode(stimuli[0]);
        //int nodesToPrint = p.length;//4;
        int nodesToPrint = 55;
        for(int yi=0; yi<nodesToPrint; yi++){

            System.out.print(p[yi]+", ");
            
        }
        System.out.println();
        p = encoder.genPopCode(stimuli[1]);       
        for(int yi=0; yi<nodesToPrint; yi++){

            System.out.print(p[yi]+", ");
            
        }
        System.out.println();
                p = encoder.genPopCode(stimuli[2]);       
        for(int yi=0; yi<nodesToPrint; yi++){

            System.out.print(p[yi]+", ");
            
        }
        System.out.println();
                p = encoder.genPopCode(stimuli[3]);       
        for(int yi=0; yi<nodesToPrint; yi++){

            System.out.print(p[yi]+", ");
            
        }
        System.out.println();
                p = encoder.genPopCode(stimuli[4]);       
        for(int yi=0; yi<nodesToPrint; yi++){

            System.out.print(p[yi]+", ");
            
        }
        System.out.println();
                p = encoder.genPopCode(stimuli[5]);       
        for(int yi=0; yi<nodesToPrint; yi++){

            System.out.print(p[yi]+", ");
            
        }
        System.out.println();
        
        
        
        
        
        
        spikes = encoder.encode(stimuli[0]);
        
        System.out.println("spike trains");
        for(int yi=0; yi<nodesToPrint; yi++){
            
            System.out.print("n"+yi+": ");
            for(int t=0; t<durationInMilSec; t++){

                System.out.print(spikes[yi][t]);
            } 
            System.out.println();
        }
       
        System.out.println("stimuli[1]");
        spikes = encoder.encode(stimuli[1]);
        
        for(int yi=0; yi<nodesToPrint; yi++){
            
            System.out.print("n"+yi+": ");
            for(int t=0; t<durationInMilSec; t++){

                System.out.print(spikes[yi][t]);
            } 
            System.out.println();
        }
       
        // get firing rate of neuron choose durationInMilSec = 1000
        double rate = 0;
        for(int i=0; i<durationInMilSec; i++){
            
            rate += spikes[3][i];
        }
        rate /= (double)durationInMilSec;
        System.out.print("rate "+rate);       
    }
    
    
    // test feature population code
    public static void test3(){
        
        // encoder parameters
        Encoder encoder = new Encoder();
        int nofRows     = 3;
        int nofCols     = 3;
        double frequencyEncoding = 40;  // 40 Hz
        double deltaT       = 0.001;    // 1 ms
        int durationInMilSec        = 50;       // ms
        int popCodeFanOut = 1;

        // orientation map parameters
        OrientationMap mapOrient = new OrientationMap(); 
        int supportRadius = 1;
        int nofScales = 1;
        double scalingFactor = 1.5;
        double orientationResolution = 30.0;
        double gaborFrequency = 1.0;
        double elongation = 0.65;

        // stimuli parameters
        int [] stimuliDims = new int[]{nofRows,nofCols};
        int nofStimuli = 6;
        double [][] stimuli = new double [ nofStimuli ][ nofRows*nofCols ];
 
        // combined initializations
        mapOrient.init(supportRadius,nofScales,scalingFactor,orientationResolution,gaborFrequency,elongation);
        
        encoder.setFeatureMap(mapOrient);
        encoder.init(frequencyEncoding, deltaT, durationInMilSec, nofRows*nofCols*mapOrient.getNofFeatureSets(), popCodeFanOut);
        encoder.setInputDimensions(stimuliDims);
        
        stimuli[0] = OrientationMapTest.genBarStimulusVals(nofRows,nofCols,nofCols/2,nofRows/2,1,90);
        stimuli[1] = OrientationMapTest.genBarStimulusVals(nofRows,nofCols,nofCols/2,nofRows/2,1,0);
        stimuli[2] = OrientationMapTest.genBarStimulusVals(nofRows,nofCols,nofCols/2,nofRows/2,1,30);
        stimuli[3] = OrientationMapTest.genBarStimulusVals(nofRows,nofCols,nofCols/2,nofRows/2,1,60);
        stimuli[4] = OrientationMapTest.genBarStimulusVals(nofRows,nofCols,nofCols/2,nofRows/2,1,120);
        stimuli[5] = OrientationMapTest.genBarStimulusVals(nofRows,nofCols,nofCols/2,nofRows/2,1,150);
        //stimuli[2] = OrientationMapTest.genBarStimulusVals(nofRows,nofCols,nofCols/2,nofRows/2,1,45);       


        int nofTimeSteps = 1;
        int nofNodes = encoder.getNofEncoderNodes();
        int [][] spikes = new int[ nofNodes ][ durationInMilSec ];
//        for(int step=0; step<nofTimeSteps; step++){
//            
//            spikes = encoder.encode(stimuli[step]);
//        }

        System.out.println("pop code");
        
        double [] p = encoder.genPopCode(stimuli[0]);
        //int nodesToPrint = p.length;//4;
        int nodesToPrint = 55;
        for(int yi=0; yi<nodesToPrint; yi++){

            System.out.print(p[yi]+", ");
            
        }
        System.out.println();
        p = encoder.genPopCode(stimuli[1]);       
        for(int yi=0; yi<nodesToPrint; yi++){

            System.out.print(p[yi]+", ");
            
        }
        System.out.println();
                p = encoder.genPopCode(stimuli[2]);       
        for(int yi=0; yi<nodesToPrint; yi++){

            System.out.print(p[yi]+", ");
            
        }
        System.out.println();
                p = encoder.genPopCode(stimuli[3]);       
        for(int yi=0; yi<nodesToPrint; yi++){

            System.out.print(p[yi]+", ");
            
        }
        System.out.println();
                p = encoder.genPopCode(stimuli[4]);       
        for(int yi=0; yi<nodesToPrint; yi++){

            System.out.print(p[yi]+", ");
            
        }
        System.out.println();
                p = encoder.genPopCode(stimuli[5]);       
        for(int yi=0; yi<nodesToPrint; yi++){

            System.out.print(p[yi]+", ");
            
        }
        System.out.println();
        
        
        
        
        
        
        spikes = encoder.encode(stimuli[0]);
        
        System.out.println("spike trains");
        for(int yi=0; yi<nodesToPrint; yi++){
            
            System.out.print("n"+yi+": ");
            for(int t=0; t<durationInMilSec; t++){

                System.out.print(spikes[yi][t]);
            } 
            System.out.println();
        }
       
        System.out.println("stimuli[1]");
        spikes = encoder.encode(stimuli[1]);
        
        for(int yi=0; yi<nodesToPrint; yi++){
            
            System.out.print("n"+yi+": ");
            for(int t=0; t<durationInMilSec; t++){

                System.out.print(spikes[yi][t]);
            } 
            System.out.println();
        }
       
        // get firing rate of neuron choose durationInMilSec = 1000
        double rate = 0;
        for(int i=0; i<durationInMilSec; i++){
            
            rate += spikes[3][i];
        }
        rate /= (double)durationInMilSec;
        System.out.print("rate "+rate);       
    }
    
    public EncoderTest(){
        

    }
    
}
