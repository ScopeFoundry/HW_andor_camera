import numpy as np
import time
import pyqtgraph as pg

from ScopeFoundry import Measurement 

#import matplotlib.gridspec as gridspec 
from time import sleep

from ScopeFoundry import h5_io

ROW0 = 240
ROW1 = 271



def pixel2wavelength(grating_position, pixel_index, binning = 1):
    # Wavelength calibration based off of work on 4/30/2014
    # changed 3/20/2015 after apd alignement offset = -5.2646 #nm
    offset = -4.2810
    focal_length = 293.50 #mm
    delta = 0.0704  #radians
    gamma = 0.6222  # radian
    grating_spacing = 1/150.  #mm
    pixel_size = 16e-3  #mm   #Binning!
    m_order = 1 #diffraction order

    wl_center = (grating_position + offset)*1e-6
    px_from_center = pixel_index*binning +binning/2. - 256
    
    psi = np.arcsin(m_order* wl_center / (2*grating_spacing*np.cos(gamma/2)))
    
    eta = np.arctan(px_from_center*pixel_size*np.cos(delta) /
    (focal_length+px_from_center*pixel_size*np.sin(delta)))
    
    return 1e6*((grating_spacing/m_order)
                    *(np.sin(psi-0.5*gamma)
                      + np.sin(psi+0.5*gamma+eta)))


class AndorCCDReadoutMeasure(Measurement):

    name = "andor_ccd_readout"
    
    def setup(self):
        
        self.display_update_period = 0.050 #seconds

        #connect events
        self.gui.ui.andor_ccd_acquire_cont_checkBox.stateChanged.connect(self.start_stop)
        self.gui.ui.andor_ccd_acq_bg_pushButton.clicked.connect(self.acquire_bg_start)
        self.gui.ui.andor_ccd_read_single_pushButton.clicked.connect(self.acquire_single_start)
        
        #local logged quantities
        self.bg_subtract = self.add_logged_quantity('bg_subtract', dtype=bool, initial=False, ro=False)
        self.acquire_bg  = self.add_logged_quantity('acquire_bg',  dtype=bool, initial=False, ro=False)
        self.read_single = self.add_logged_quantity('read_single', dtype=bool, initial=False, ro=False)
        
        self.bg_subtract.connect_bidir_to_widget(self.gui.ui.andor_ccd_bgsub_checkBox)
    
    def acquire_bg_start(self):
        self.acquire_bg.update_value(True)
        self.start()
    
    def acquire_single_start(self):
        self.read_single.update_value(True)
        self.start()

    def setup_figure(self):
        #Andor CCD data
        """self.fig_ccd_image = self.gui.add_figure('ccd_image', self.gui.ui.plot_andor_ccd_widget)
        self.fig_ccd_image.clf()
        """

        if hasattr(self, 'graph_layout'):
            self.graph_layout.deleteLater() # see http://stackoverflow.com/questions/9899409/pyside-removing-a-widget-from-a-layout
            del self.graph_layout
            
        self.graph_layout=pg.GraphicsLayoutWidget(border=(100,100,100))
        self.gui.ui.plot_andor_ccd_widget.layout().addWidget(self.graph_layout)
        
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
    
        #setup data arrays         
        
        ccd = self.gui.andor_ccd_hc.andor_ccd
        
        width_px = ccd.Nx_ro
        height_px = ccd.Ny_ro
        
        
        t_acq = self.gui.andor_ccd_hc.exposure_time.val #in seconds
        
        wait_time = 0.01 #np.min(1.0,np.max(0.05*t_acq, 0.05)) # limit update period to 50ms (in ms) or as slow as 1sec
        
        try:
            self.log.info("starting acq")
            ccd.start_acquisition()
        
            self.log.info( "checking..." )
            t0 = time.time()
            while not self.interrupt_measurement_called:
            
                self.wls  = pixel2wavelength(self.gui.acton_spec_hc.center_wl.val, 
                              np.arange(width_px), binning=ccd.get_current_hbin())

                stat = ccd.get_status()
                if stat == 'IDLE':
                    # grab data
                    t1 = time.time()
                    #print "acq time", (t1-t0)
                    t0 = t1
                
                
                    self.buffer_ = ccd.get_acquired_data()

                
                    if self.bg_subtract.val and not self.acquire_bg.val:
                        bg = self.gui.andor_ccd_hc.background
                        if bg is not None:
                            if bg.shape == self.buffer_.shape:
                                self.buffer_ = self.buffer_ - bg
                            else:
                                self.log.warning("Background not the correct shape {} != {}".format( self.buffer_.shape, bg.shape))
                        else:
                            self.log.warning( "No Background available, raw data shown")
                    self.spectra_data = np.average(self.buffer_, axis=0)
 
 
                    if self.acquire_bg.val or self.read_single.val:
                        break # end the while loop for non-continuous scans
                    else:
                        # restart acq
                        ccd.start_acquisition()
                    
                else:
                    #sleep(wait_time)
                    sleep(0.01)
        except Exception as err:
            self.log.error( "{} error: {}".format(self.name, err))
        finally:            
            # while-loop is complete
            self.gui.andor_ccd_hc.interrupt_acquisition()

            
            #is this right place to put this?
            # Signal emission from other threads ok?
            #self.measurement_state_changed.emit(False)
        
        
            if self.acquire_bg.val:
                if self.interrupt_measurement_called:
                    self.gui.andor_ccd_hc.background = None
                else:
                    self.gui.andor_ccd_hc.background = self.buffer_.copy()
                self.acquire_bg.update_value(False)    
        
            if self.read_single.val:
                if self.interrupt_measurement_called:
                    self.spectrum = None
                else:
                    self.spectrum = self.buffer_.copy()
        
                save_dict = {
                         'spectrum': self.spectrum,
                         'wls': self.wls,
                            }               
                
                for lqname,lq in self.gui.logged_quantities.items():
                    save_dict[lqname] = lq.val
                for hc in self.gui.hardware_components.values():
                    for lqname,lq in hc.logged_quantities.items():
                        save_dict[hc.name + "_" + lqname] = lq.val
                for lqname,lq in self.logged_quantities.items():
                    save_dict[self.name +"_"+ lqname] = lq.val

                self.fname = "%i_%s.npz" % (time.time(), self.name)
                np.savez_compressed(self.fname, **save_dict)
                self.log.info( "saved: " + self.fname)
                
                self.log.info( "Andor CCD single acq successfully acquired")
                self.read_single.update_value(False)    

            # Send completion signals
            if not self.interrupt_measurement_called:
                self.measurement_sucessfully_completed.emit()
            else:
                self.measurement_interrupted.emit()
    
    def update_display(self):

        self.img_item.setImage(self.buffer_.astype(np.float32).T, autoLevels=False)
        self.hist_lut.imageChanged(autoLevel=True, autoRange=True)

        self.spec_plot_line.setData(self.wls, self.spectra_data)


class AndorCCDStepAndGlue(Measurement):

    name = "andor_ccd_step_and_glue"
    
    def setup(self):
        
        self.display_update_period = 0.050 #seconds

        
        #local logged quantities
        self.bg_subtract = self.add_logged_quantity('bg_subtract', dtype=bool, initial=False, ro=False)
        

        self.center_wl_start = self.add_logged_quantity('center_wl_start', dtype=float, initial=400, ro=False)
        self.center_wl_stop  = self.add_logged_quantity('center_wl_stop', dtype=float, initial=1100, ro=False)
        self.center_wl_step = self.add_logged_quantity('center_wl_step', dtype=float, initial=50, ro=False)

        
        #connect events
        self.bg_subtract.connect_bidir_to_widget(self.gui.ui.andor_ccd_bgsub_checkBox)
        


    def _run(self):

        # Hardware
        ccd = self.gui.andor_ccd_hc.andor_ccd
        acton_spec_hc = self.gui.acton_spec_hc
    
        width_px = ccd.Nx_ro
        height_px = ccd.Ny_ro
        
        #ccd_shape=(self.Ny_ro, self.Nx_ro)

        # h5 data file setup
        self.t0 = time.time()
        self.h5_file = h5_io.h5_base_file(self.gui, "%i_%s.h5" % (self.t0, self.name) )
        self.h5_file.attrs['time_id'] = self.t0
        h5m = self.h5_meas_group = self.h5_file.create_group(self.name)
        
        h5m.attrs['time_id'] = self.t0
        h5_io.h5_save_measurement_settings(self, h5m)
        h5_io.h5_save_hardware_lq(self.gui, h5m)

        #setup data arrays (in h5 file)
        
        # center wl array contains start and stop centers         
        self.center_wl_array = np.arange(self.center_wl_start.val, 
                                         self.center_wl_stop.val + self.center_wl_step.val,
                                         self.center_wl_step.val)
        num_specs = len(self.center_wl_array)
        self.center_wl_array = h5m.create_dataset('center_wl_array', data=self.center_wl_array)
        self.center_wl_actual = h5m.create_dataset('center_wl_actual', shape=self.center_wl_array.shape)
        
        self.wls = h5m.create_dataset('wls', (num_specs, width_px), dtype=float)
        self.spectra_data = h5m.create_dataset('spectra_data', 
                                               shape=(num_specs, height_px, width_px,),
                                               dtype=np.int32, compression='gzip')
        
        if self.bg_subtract.val:
            bg = self.bg = self.gui.andor_ccd_hc.background
            if bg is not None and bg.shape == ccd.buffer.shape:
                h5m['andor_ccd_bg'] = self.bg
            else:
                self.log.warning( "Background not avail or the correct shape {}".format(ccd.buffer.shape))#,# bg#bg.shape
                self.bg_subtract.update_value(False)
        try:
            for ii, center_wl in enumerate(self.center_wl_array):
                if self.interrupt_measurement_called:
                    break
                
                #TODO add progress update

                # move to center wl
                acton_spec_hc.center_wl.update_value(center_wl)
                self.center_wl_actual[ii] = acton_spec_hc.center_wl.val 
                
                self.wls[ii,:]  = pixel2wavelength(acton_spec_hc.center_wl.val, 
                                             np.arange(width_px), binning=ccd.get_current_hbin())

                self.log.info("starting ccd acq")
                ccd.start_acquisition()
                
                # wait until ccd is done acquiring
                while True:
                    if self.interrupt_measurement_called:
                        break
                    stat = ccd.get_status()
                    if stat == 'IDLE':
                        self.ccd_buffer = ccd.get_acquired_data()
    
                        if self.bg_subtract.val:
                            self.ccd_buffer = self.ccd_buffer - self.bg
                            self.log.debug("self.bg.shape {}".format(self.bg.shape))
                            self.log.debug("self.ccd_buffer.shape {}".format(self.ccd_buffer.shape))
                            

                        self.spectra_data[ii,:,:] = self.ccd_buffer
                        break
                    else:
                        sleep(0.01) # wait for a while before polling
                self.h5_file.flush()
                        
        except Exception as err:
            self.log.error( "{} error: {}".format(self.name, err))
        finally:            
            self.gui.andor_ccd_hc.interrupt_acquisition()
            self.h5_file.close()