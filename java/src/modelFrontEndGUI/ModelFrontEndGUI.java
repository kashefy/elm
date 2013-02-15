/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

/*
 * ModelFrontEndGUI.java
 *
 * Created on Oct 13, 2011, 8:46:41 PM
 */
package modelFrontEndGUI;

/**
 *
 * @author woodstock
 */
// jdk imports
import javax.swing.*;
import java.awt.image.*;
import java.awt.Graphics;

import java.io.*;
import javax.imageio.*;

// sst imports
import org.shared.array.*;
import org.shared.image.filter.Gabor;

// own imports
import model.utils.ModelUtils;
import model.utils.files.FileIO;
import model.*;
import model.encoding.*;
import model.features.OrientationMapTest;
import model.utils.StdRandom;
import model.utils.files.DataLoaderTest;
import model.simulation.*;
import model.evaluation.*;
import model.features.IntensityContrastMap;
import model.features.DiffOfGaussians2DSq;
import model.features.IntensityContrastMapTest;
import model.utils.Distribution2D;
import model.attention.*;
import model.utils.Distribution1D;
import model.utils.files.DataLoaderImageSetCSV_incremental;

public class ModelFrontEndGUI extends javax.swing.JFrame {

    /** Creates new form ModelFrontEndGUI */
    public ModelFrontEndGUI() {
        initComponents();
        
        //randomTests();
              
        //new OrientationMapTest().test1();
        //new OrientationMapTest().test2();
        //new OrientationMapTest().test3();
        //OrientationMapTest.testFilterResponseBarStim();
        //OrientationMapTest.testFilterResponseBarStimWithNoise();
        //OrientationMapTest.testFilterResponseMNIST();
        //OrientationMapTest.testCalcNeighOrientDist();
        //new DiffOfGaussians2DSq().test();
        //IntensityContrastMapTest.test1();
        //IntensityContrastMapTest.test2();
        //IntensityContrastMapTest.test3();
        
        //new EncoderTest().test1();
        //new EncoderTest().test2();
        
        //new ModelPredictionTest().test1();
        //new ModelPredictionTest().test2();
        //new ModelPredictionTest().test3();
        //ModelPredictionTest.test4();
        //ModelPredictionTest.test4Timed();
        //ModelPredictionTest.test5();
        //new ModelPredictionTest().testOnMNIST();
        
        //new Histogram().testRun();
        //EvaluationTest.test_predictionStats_window();
        
        
        //StdRandom.test1();
        //ModelUtils.testPad2();
        //ModelUtils.testExtractColumn();
        //DataLoaderTest.testLoadMNISTCSV();
        //DataLoaderTest.testTranslation();
        //ModelUtils.testNextPow2();
        //ModelUtils.testPad3();
        //new Distribution2D().test();
        //new Distribution1D().test();
        //ModelUtils.testNeighVar();
        //DataLoaderImageSetCSV_incremental.test_tree_set();
        
        //double [] x = new double[]{1,10,100,1000};
        //double [] y = ModelUtils.cumulativeSum(x);
        
        //new PopulationCodeTest().testMAXPopCode();
        //new PopulationCodeTest().testSoftMAXPopCode();
        //FeaturePopulationCode.testEncodeViaProb();
        
        //SimulationTest.testSimplePatterns();
        //SimulationTest.testMNIST();
        //SimulationTest.testAttSeq();
        //SimulationTest.testMNISTInterm();
        //SimulationTest.testMNIST_layerF();
        //SimulationTest.testMNIST_layerF_onOff();
        //SimulationTest.test_MNIST_layerIndep();
        
        //SimulationTune.tune_MNIST_layerF_onOff();
//        SimulationTune.tune_MNIST_layerF_onOff_learningRateF();
//        SimulationTune.tune_MNIST_layerF_onOff_learningRateZ();
//        SimulationTune.tune_MNIST_layerF_onOff_wta_max_rate_factor_layerF();
//        SimulationTune.tune_MNIST_layerF_onOff_wta_max_rate_factor_layerZ();
//        
        //SaliencyTest.testEval();
        //AttentionTest.test_attentionSalient();
        //AttentionTest.test_attentionWindow_filterResponse_MNIST();
        //ActivityMaskTest.test_activity_sub_windows();
       
         // for debugging purposes
        int abc = 0;
        abc += 1;
    }

    /** This method is called from within the constructor to
     * initialize the form.
     * WARNING: Do NOT modify this code. The content of this method is
     * always regenerated by the Form Editor.
     */
    @SuppressWarnings("unchecked")
    // <editor-fold defaultstate="collapsed" desc="Generated Code">//GEN-BEGIN:initComponents
    private void initComponents() {

        jButtonAction = new javax.swing.JButton();
        jLabelStatus = new javax.swing.JLabel();
        jLabelImgDisplay = new javax.swing.JLabel();

        setDefaultCloseOperation(javax.swing.WindowConstants.EXIT_ON_CLOSE);

        jButtonAction.setText("Action");
        jButtonAction.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                jButtonActionActionPerformed(evt);
            }
        });

        jLabelStatus.setText("Status");

        javax.swing.GroupLayout layout = new javax.swing.GroupLayout(getContentPane());
        getContentPane().setLayout(layout);
        layout.setHorizontalGroup(
            layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGroup(layout.createSequentialGroup()
                .addGroup(layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                    .addGroup(layout.createSequentialGroup()
                        .addContainerGap()
                        .addComponent(jButtonAction)
                        .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.UNRELATED)
                        .addComponent(jLabelStatus))
                    .addGroup(layout.createSequentialGroup()
                        .addGap(24, 24, 24)
                        .addComponent(jLabelImgDisplay)))
                .addContainerGap(273, Short.MAX_VALUE))
        );
        layout.setVerticalGroup(
            layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGroup(layout.createSequentialGroup()
                .addContainerGap()
                .addGroup(layout.createParallelGroup(javax.swing.GroupLayout.Alignment.BASELINE)
                    .addComponent(jButtonAction)
                    .addComponent(jLabelStatus))
                .addGap(18, 18, 18)
                .addComponent(jLabelImgDisplay)
                .addContainerGap(244, Short.MAX_VALUE))
        );

        pack();
    }// </editor-fold>//GEN-END:initComponents

    private void jButtonActionActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_jButtonActionActionPerformed
        // TODO add your handling code here:
        jLabelStatus.setText("Running");
        System.exit(0);
    }//GEN-LAST:event_jButtonActionActionPerformed

    /**
     * @param args the command line arguments
     */
    public static void main(String args[]) {
        /* Set the Nimbus look and feel */
        //<editor-fold defaultstate="collapsed" desc=" Look and feel setting code (optional) ">
        /* If Nimbus (introduced in Java SE 6) is not available, stay with the default look and feel.
         * For details see http://download.oracle.com/javase/tutorial/uiswing/lookandfeel/plaf.html 
         */
        try {
            for (javax.swing.UIManager.LookAndFeelInfo info : javax.swing.UIManager.getInstalledLookAndFeels()) {
                if ("Nimbus".equals(info.getName())) {
                    javax.swing.UIManager.setLookAndFeel(info.getClassName());
                    break;
                }
            }
        } catch (ClassNotFoundException ex) {
            java.util.logging.Logger.getLogger(ModelFrontEndGUI.class.getName()).log(java.util.logging.Level.SEVERE, null, ex);
        } catch (InstantiationException ex) {
            java.util.logging.Logger.getLogger(ModelFrontEndGUI.class.getName()).log(java.util.logging.Level.SEVERE, null, ex);
        } catch (IllegalAccessException ex) {
            java.util.logging.Logger.getLogger(ModelFrontEndGUI.class.getName()).log(java.util.logging.Level.SEVERE, null, ex);
        } catch (javax.swing.UnsupportedLookAndFeelException ex) {
            java.util.logging.Logger.getLogger(ModelFrontEndGUI.class.getName()).log(java.util.logging.Level.SEVERE, null, ex);
        }
        //</editor-fold>

        /* Create and display the form */
        java.awt.EventQueue.invokeLater(new Runnable() {

            public void run() {
              
                ModelFrontEndGUI mainFrame = new ModelFrontEndGUI();
                mainFrame.setVisible(true);
                mainFrame.pack();
            }
        });
        
        
    }
    // Variables declaration - do not modify//GEN-BEGIN:variables
    private javax.swing.JButton jButtonAction;
    private javax.swing.JLabel jLabelImgDisplay;
    private javax.swing.JLabel jLabelStatus;
    // End of variables declaration//GEN-END:variables
}
