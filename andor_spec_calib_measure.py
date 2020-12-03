from ScopeFoundry import Measurement, h5_io
from ScopeFoundry.helper_funcs import sibling_path, load_qt_ui_file,\
    replace_spinbox_in_layout

import pyqtgraph as pg
import numpy as np
import time

class AndorSpecCalibMeasure(Measurement):
    
    name = 'andor_spec_calib'
    
    def setup(self):
        
        self.settings.New_Range('sweep_wls', dtype=float)
        
        self.ui_filename = sibling_path(__file__, 'andor_spec_calib_measure.ui')
        self.ui = load_qt_ui_file(self.ui_filename)
        
    def setup_figure(self):

        self.graph_layout=pg.GraphicsLayoutWidget()
        self.ui.plot_widget.layout().addWidget(self.graph_layout)
        
        self.img_plot = self.graph_layout.addPlot()
        self.img_plot.showGrid(x=True, y=True)
        self.img_item = pg.ImageItem()
        self.img_plot.addItem(self.img_item)

        self.hist_lut = pg.HistogramLUTItem()
        self.hist_lut.autoHistogramRange()
        self.hist_lut.setImageItem(self.img_item)
        self.graph_layout.addItem(self.hist_lut)


        self.graph_layout.nextRow()
        
        self.spectrum_plot = self.graph_layout.addPlot(
            title="Spectrum", colspan=2)        
        self.current_spec_plotline = self.spectrum_plot.plot()


        # start stop buttons
        self.ui.start_pushButton.clicked.connect(
            self.start)
        self.ui.interrupt_pushButton.clicked.connect(
            self.interrupt)


        # WL sweep controls
        self.settings.sweep_wls_min.connect_to_widget(
            self.ui.sweep_wls_min_doubleSpinBox)
        self.settings.sweep_wls_max.connect_to_widget(
            self.ui.sweep_wls_max_doubleSpinBox)
        self.settings.sweep_wls_step.connect_to_widget(
            self.ui.sweep_wls_step_doubleSpinBox)
        self.settings.sweep_wls_num.connect_to_widget(
            self.ui.sweep_wls_num_doubleSpinBox)
        
        # Camera settings
        self.andor_ccd = self.app.hardware['andor_ccd']
        self.andor_ccd.settings.em_gain.connect_to_widget(
            self.ui.andor_emgain_doubleSpinBox)
        self.andor_ccd.settings.exposure_time.connect_to_widget(
            self.ui.andor_exp_time_doubleSpinBox)
        
        # Spectrometer settings
        if 'acton_spectrometer' in list(self.app.hardware.keys()):
            self.spec = spec = self.app.hardware['acton_spectrometer']
            spec.settings.entrance_slit.connect_to_widget(self.ui.spec_ent_slit_doubleSpinBox)
        elif 'andor_spec' in list(self.app.hardware.keys()):
            self.spec = spec = self.app.hardware['andor_spec']
            spec.settings.slit_input_side.connect_to_widget(self.ui.spec_ent_slit_doubleSpinBox)
        else:
            raise Exception('No spectrometer!')
        spec.settings.center_wl.connect_to_widget(
            self.ui.spec_center_wl_doubleSpinBox)
        
        spec.settings.grating_id.connect_to_widget(
            self.ui.spec_grating_id_comboBox)


    def run(self):
        # Set up Hardware        
        self.andor_ccd.settings['acq_mode'] = 'single'
        self.andor_ccd.settings['trigger_mode'] = 'internal'
        self.andor_ccd.set_readout()
        
        ccd_hw = self.app.hardware['andor_ccd']
        ccd_dev = ccd_hw.ccd_dev
        
        width_px = ccd_dev.Nx_ro
        height_px = ccd_dev.Ny_ro

        try:
            # create data file and array
            self.h5_file = h5_io.h5_base_file(app=self.app, measurement=self)
            self.h5m = h5_io.h5_create_measurement_group(measurement=self, 
                                                         h5group=self.h5_file)
            
            self.sweep_wls = self.settings.ranges['sweep_wls'].array
            self.h5m['sweep_wls'] = self.sweep_wls
            
            self.spectra = np.zeros((len(self.sweep_wls), width_px), dtype=float)
            self.spectra_h5 = self.h5m.create_dataset('spectra', 
                                                      shape=(len(self.sweep_wls), width_px),
                                                      dtype=float)
            
            for ii, center_wl in enumerate(self.sweep_wls):
                if self.interrupt_measurement_called:
                    break
                # move spectrometer to center wavelength
                self.spec.settings['center_wl'] = center_wl
                
                ccd_dev.start_acquisition()
    
                stat = ccd_hw.settings.ccd_status.read_from_hardware()
                while stat == 'ACQUIRING':
                    if self.interrupt_measurement_called:
                        break
                    time.sleep(0.01)
                    stat = ccd_hw.settings.ccd_status.read_from_hardware()
    
                if stat == 'IDLE':
                    self.ccd_img = ccd_dev.get_acquired_data()
                    self.spectrum = np.average(self.ccd_img, axis=0)
                    self.spectra[ii,:] = self.spectrum
                    self.spectra_h5[ii,:] = self.spectrum

        finally:
            print(self.name, 'done')
            self.h5_file.close()
        
    def update_display(self):
        self.img_item.setImage(self.spectra.T)
        self.current_spec_plotline.setData(self.spectrum)
        
if __name__ == '__main__':
    import sys
    from ScopeFoundry import BaseMicroscopeApp
    
    class TestApp(BaseMicroscopeApp):
        
        def setup(self):
            from ScopeFoundryHW.andor_camera import AndorCCDHW
            self.add_hardware(AndorCCDHW(self))
            from ScopeFoundryHW.acton_spec import ActonSpectrometerHW
            self.add_hardware(ActonSpectrometerHW(self))
            self.add_measurement(AndorSpecCalibMeasure(self))
    app = TestApp(sys.argv)
    app.exec_()