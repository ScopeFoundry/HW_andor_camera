from ScopeFoundry import Measurement
from ScopeFoundry import h5_io
from ScopeFoundry.helper_funcs import load_qt_ui_file, sibling_path
import pyqtgraph as pg
import numpy as np
import time

class AndorCCDKineticMeasure(Measurement):
    
    name = 'andor_ccd_kinetic'
    
    def setup(self):
        pass
    
    def setup_figure(self):
        ui = self.ui = load_qt_ui_file(sibling_path(__file__, 'andor_ccd_readout.ui'))
        
        ## ui connection
        andor = self.app.hardware['andor_ccd']
        andor.settings.exposure_time.connect_to_widget(ui.andor_ccd_int_time_doubleSpinBox)
        andor.settings.em_gain.connect_to_widget(ui.andor_ccd_emgain_doubleSpinBox)
        andor.settings.temperature.connect_to_widget(ui.andor_ccd_temp_doubleSpinBox)
        andor.settings.ccd_status.connect_to_widget(ui.andor_ccd_status_label)
        andor.settings.shutter_open.connect_to_widget(ui.andor_ccd_shutter_open_checkBox)
        
        #self.settings.bg_subtract.connect_to_widget(ui.andor_ccd_bgsub_checkBox)
        ui.andor_ccd_acquire_cont_checkBox.stateChanged.connect(self.start_stop)
        #ui.andor_ccd_acq_bg_pushButton.clicked.connect(self.acquire_bg_start)
        #ui.andor_ccd_read_single_pushButton.clicked.connect(self.acquire_single_start)


        #### PLot window
        self.graph_layout = pg.GraphicsLayoutWidget()
        self.ui.plot_groupBox.layout().addWidget(self.graph_layout)
        
        self.spec_plot = self.graph_layout.addPlot()
        self.spec_plot_line = self.spec_plot.plot([1,3,2,4,3,5])
        self.spec_plot.enableAutoRange()
        
        
        self.graph_layout.nextRow()
        
        self.img_plot = self.graph_layout.addPlot()
        #self.img_plot.getViewBox().setLimits(minXRange=-10, maxXRange=100, minYRange=-10, maxYRange=100)
        self.img_plot.showGrid(x=True, y=True)
        self.img_plot.setAspectLocked(lock=True, ratio=1)
        self.img_item = pg.ImageItem()
        self.img_plot.addItem(self.img_item)


        self.hist_lut = pg.HistogramLUTItem()
        self.hist_lut.autoHistogramRange()
        self.hist_lut.setImageItem(self.img_item)
        self.graph_layout.addItem(self.hist_lut)
    
    def run(self):
        ccd_hw = self.app.hardware['andor_ccd']
        ccd_dev = ccd_hw.ccd_dev

        N = ccd_dev.get_num_kinetics()
        width_px = ccd_dev.Nx_ro
        height_px = ccd_dev.Ny_ro
        
        try:
            ccd_dev.start_acquisition()

            stat = ccd_hw.settings.ccd_status.read_from_hardware()
            
            while stat == 'ACQUIRING':
                print(stat, "GetTotalNumberImagesAcquired",
                      ccd_dev.get_total_number_images_acquired())
                
                time.sleep(0.1)
                stat = ccd_hw.settings.ccd_status.read_from_hardware()
                if self.interrupt_measurement_called:
                    ccd_hw.interrupt_acquisition()
                    break
                
        finally:
            print("done")
