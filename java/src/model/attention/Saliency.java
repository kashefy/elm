/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package model.attention;

import model.features.*;
import model.encoding.FeaturePopulationCode;
import model.utils.Distribution2D;
import org.shared.array.RealArray;
import org.shared.array.ComplexArray;
import model.utils.ModelUtils;
import model.utils.files.FileIO;

import java.io.File;

/**
 *
 * @author woodstock
 */
public class Saliency extends AbstractSaliencyMeasure{
    
    public static int FEAT_MAP_INDEX_ORIENT = 0;
    public static int FEAT_MAP_INDEX_INTENSCONTRAST = 1;
    
    int m_nofFeatureMaps;
    
    FeatureMapParams m_featMapParamsRef;
    
    AbstractFeatureMap [] m_arrFeatMaps;
    OrientationMap m_orientMap;
    FeaturePopulationCode m_featPopCoder;
    
    int m_nofRows;
    int m_nofCols;
    double m_cutOffBelow_orientResponseVals;
    RealArray m_saliency;
    Distribution2D m_saliencyDistr;
    
    RealArray m_orientDistsVarNorm;
    RealArray m_arrResponseIntensContrastRect_0;
    
    @Override
    public void setParams(SaliencyParams par_params){
        
        m_featMapParamsRef = par_params.m_featMapParams;
        m_nofRows = par_params.m_nofRows;
        m_nofCols = par_params.m_nofCols;
        m_cutOffBelow_orientResponseVals = par_params.m_cutOffBelow_orientResponseVals;
    }
    
    @Override
    public void init(){
        
        m_nofFeatureMaps = 2;
        m_arrFeatMaps = new AbstractFeatureMap[ m_nofFeatureMaps ];

        m_orientMap = new OrientationMap();
        m_arrFeatMaps[ FEAT_MAP_INDEX_ORIENT ] = m_orientMap;
        m_arrFeatMaps[ FEAT_MAP_INDEX_ORIENT ].setParams(m_featMapParamsRef);
        m_arrFeatMaps[ FEAT_MAP_INDEX_ORIENT ].init();
        m_featPopCoder = new FeaturePopulationCode();
        m_featPopCoder.setFeatureMap(m_arrFeatMaps[ FEAT_MAP_INDEX_ORIENT ]);
        m_featPopCoder.init(m_nofRows*m_nofCols*m_arrFeatMaps[ FEAT_MAP_INDEX_ORIENT ].getNofFeatureSets(), 1);
        m_featPopCoder.setInputDimensions(new int []{m_nofRows, m_nofCols});
        
        m_arrFeatMaps[ FEAT_MAP_INDEX_INTENSCONTRAST ] = new IntensityContrastMap();
        m_arrFeatMaps[ FEAT_MAP_INDEX_INTENSCONTRAST ].setParams(m_featMapParamsRef);
        m_arrFeatMaps[ FEAT_MAP_INDEX_INTENSCONTRAST ].init();
        
        m_saliencyDistr = new Distribution2D();
        m_saliencyDistr.setParams(m_nofRows, m_nofCols);
        m_saliencyDistr.init();
    }
    
    @Override
    public void eval(double [] par_inputVals){

        double [] stimulusValsComplex;
         stimulusValsComplex = ModelUtils.prepRealValuesForComplex(par_inputVals);
        int [] dimsComplex = new int[]{m_nofRows, m_nofCols, 2};
        ComplexArray stimulus = new ComplexArray(stimulusValsComplex, dimsComplex);
        
        m_saliency = new RealArray(m_nofRows, m_nofCols);
        
        // orientation
        double [] popCodeStateValsDistr;
        popCodeStateValsDistr = m_featPopCoder.calcStateOfNeuronsDistr(par_inputVals);
        int [][] spikeTrains;
        int nofSamples = 1;
        spikeTrains = m_featPopCoder.sampleStateOfNeurons(popCodeStateValsDistr, nofSamples);
        int nof_pixels = par_inputVals.length;
        double [] orientVals = new double [ nof_pixels ];
        double [] orientResponseVals = new double [ nof_pixels ];
        int nofOrientations = m_arrFeatMaps[ FEAT_MAP_INDEX_ORIENT ].getNofFeatureSets();
        double orientResolution = 180.0/(double)nofOrientations;
        int j=0;
        for(int i=0; i<nof_pixels; ++i){
            
            int nodeOffset = i*nofOrientations;
            boolean b_found_spike = false;
            for(int featIndex = 0; featIndex < nofOrientations && !b_found_spike; ++featIndex){
                
                b_found_spike = spikeTrains[ nodeOffset + featIndex ][ nofSamples-1 ] == 1;
                if(b_found_spike){
                    orientVals[i] = featIndex*orientResolution;
                    orientResponseVals[i] = popCodeStateValsDistr[  nodeOffset + featIndex ];
                }
            }
        }
        
        RealArray orients = new RealArray(orientVals,m_nofRows, m_nofCols);
        RealArray orientDists = OrientationMap.calcNeighOrientDist(orients, 3);
        RealArray orientDistsVar = ModelUtils.calcNeighVar(orientDists, 3);
        RealArray orientDistsVarNorm = ModelUtils.normalizeMm(orientDistsVar, 3, 1.0);
        
        // clip away low response pixels
        RealArray orientResponse = new RealArray(orientResponseVals, m_nofRows, m_nofCols);
        orientResponse = ModelUtils.normalizeMm(orientResponse, 3, 1.0);
        orientResponse = orientResponse.uAdd(-orientResponse.aMin());
        orientResponse = orientResponse.uMul(1.0/orientResponse.aMax());
        //orientResponse = ModelUtils.normalizeMm(orientResponse, 3, 1.0);
        for(int i=0; i<nof_pixels; ++i){
            
            if(orientResponseVals[i] < m_cutOffBelow_orientResponseVals)
               orientDistsVarNorm.values()[i] = 0; 
        }
        orientDistsVarNorm = orientDistsVarNorm.uMul(1.0/orientDistsVarNorm.aMax());
        
        //m_saliency = new RealArray(orientDistsVarNorm.dims());
        m_saliency = m_saliency.eAdd(orientDistsVarNorm);
        
        // intensity contrast
        m_arrFeatMaps[ FEAT_MAP_INDEX_INTENSCONTRAST ].setStimulus(stimulus);
        ComplexArray [] arrResponseIntensContrast = m_arrFeatMaps[ FEAT_MAP_INDEX_INTENSCONTRAST ].convolve();
        RealArray [] arrResponseIntensContrastRect = new RealArray[ m_arrFeatMaps[ FEAT_MAP_INDEX_INTENSCONTRAST ].getNofFeatureSets() ];
        for(int i=0; i<m_arrFeatMaps[ FEAT_MAP_INDEX_INTENSCONTRAST ].getNofFeatureSets(); ++i){
            
            arrResponseIntensContrastRect[i] = IntensityContrastMap.rectify(arrResponseIntensContrast[i].torRe());
            arrResponseIntensContrastRect[i] = ModelUtils.normalizeMm(arrResponseIntensContrastRect[i], 3, 1.0);
            arrResponseIntensContrastRect[i] = arrResponseIntensContrastRect[i].uMul(1.0/arrResponseIntensContrastRect[i].aMax());
            m_saliency = m_saliency.eAdd(arrResponseIntensContrastRect[i]);
        }
        
        m_saliencyDistr.evalDistr(m_saliency.values());

        // for later saveing
        m_orientDistsVarNorm = orientDistsVarNorm;
        m_arrResponseIntensContrastRect_0 = arrResponseIntensContrastRect[0];
        //String strOutputPath = "C:\\Users\\woodstock\\Documents\\grad\\Thesis\\code\\sem\\java\\data\\output\\MNIST\\filterResponse\\";
        
        //FileIO.saveArrayToCSV(par_inputVals, m_nofRows, m_nofCols, strOutputPath + "stimRe.csv");
        //FileIO.saveArrayToCSV(orientDistsVarNorm.values(), m_nofRows, m_nofCols, strOutputPath + "orientDistsVarNorm.csv");
        //FileIO.saveArrayToCSV(arrResponseIntensContrastRect[0].values(), m_nofRows, m_nofCols, strOutputPath + "intens.csv");
        //FileIO.saveArrayToCSV(m_saliency.values(), m_nofRows, m_nofCols, strOutputPath + "saliency.csv");
    }
    
    @Override
    public double measure(int [] par_locRef){
        
        int nofSamples = 1;
        int [][] samples;
        samples = m_saliencyDistr.sample(nofSamples);
        System.arraycopy(samples[0], 0, par_locRef, 0, 2);
        return m_saliency.get(par_locRef);
    }
    
    @Override
    public double [] measure(int [][] par_arr_locRef){
        
        int nofSamples = par_arr_locRef.length;
        int [][] samples;
        double [] arr_saliencyVals = new double[ nofSamples ];
        samples = m_saliencyDistr.sample(nofSamples);
        for(int i=0; i<nofSamples; ++i){
            
            System.arraycopy(samples[i], 0, par_arr_locRef[i], 0, 2);
            arr_saliencyVals[i] = m_saliency.get(par_arr_locRef[i]);
        }
        
        return arr_saliencyVals;
    }
    
    @Override
    public double[] getSaliency(){
        
        int nofVals = m_saliency.values().length;
        double[] saliencyValsToExport = new double [ nofVals ];
        System.arraycopy(m_saliency.values(), 0, saliencyValsToExport, 0, nofVals);
        return saliencyValsToExport;
    }
    
    @Override
    public void save(String par_strPath){
        
        File output_dir = new File(par_strPath);
        if( output_dir.isDirectory()) {
            
            //FileIO.saveArrayToCSV(par_inputVals, m_nofRows, m_nofCols, strOutputPath + "stimRe.csv");
            FileIO.saveArrayToCSV(m_orientDistsVarNorm.values(), m_nofRows, m_nofCols, new File(output_dir, "orientDistsVarNorm.csv").getPath());
            FileIO.saveArrayToCSV(m_arrResponseIntensContrastRect_0.values(), m_nofRows, m_nofCols, new File(output_dir, "intens.csv").getPath());
            FileIO.saveArrayToCSV(m_saliency.values(), m_nofRows, m_nofCols, new File(output_dir, "saliency.csv").getPath());
        }
        else{
            System.err.println(par_strPath + " is not a directory.");
        }
    }
}
