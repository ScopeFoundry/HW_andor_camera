import numpy as np
import time
import pyqtgraph as pg

from ScopeFoundry import Measurement 

from time import sleep

from ScopeFoundry import h5_io
from ScopeFoundry.helper_funcs import load_qt_ui_file, sibling_path

# ROW0 = 240
# ROW1 = 271
# 
# 
# 
# def pixel2wavelength(grating_position, pixel_index, binning = 1):
#     # Wavelength calibration based off of work on 4/30/2014
#     # changed 3/20/2015 after apd alignement offset = -5.2646 #nm
#     offset = -4.2810
#     focal_length = 293.50 #mm
#     delta = 0.0704  #radians
#     gamma = 0.6222  # radian
#     grating_spacing = 1/150.  #mm
#     pixel_size = 16e-3  #mm   #Binning!
#     m_order = 1 #diffraction order
# 
#     wl_center = (grating_position + offset)*1e-6
#     px_from_center = pixel_index*binning +binning/2. - 256
#     
#     psi = np.arcsin(m_order* wl_center / (2*grating_spacing*np.cos(gamma/2)))
#     
#     eta = np.arctan(px_from_center*pixel_size*np.cos(delta) /
#     (focal_length+px_from_center*pixel_size*np.sin(delta)))
#     
#     return 1e6*((grating_spacing/m_order)
#                     *(np.sin(psi-0.5*gamma)
#                       + np.sin(psi+0.5*gamma+eta)))


class AndorCCDReadoutMeasure(Measurement):

    name = "andor_ccd_readout"
    
    def setup(self):
        
        self.display_update_period = 0.050 #seconds

        #local logged quantities
        self.bg_subtract = self.settings.New('bg_subtract', dtype=bool, initial=False, ro=False)
        self.acquire_bg  = self.settings.New('acquire_bg',  dtype=bool, initial=False, ro=False)
        self.count_rate = self.settings.New('count_rate', initial=0, ro=True)

        self.settings.New('continuous', dtype=bool, initial=True, ro=False) 
        self.settings.New('save_h5', dtype=bool, initial=True)

        self.settings.New('wl_calib', dtype=str, initial='pixels', choices=('pixels','raw_pixels','acton_spectrometer', 'andor_spectrometer'))
        self.settings.New('explore_mode_exposure_time', initial=0.1, unit='sec', spinbox_decimals=4,
                          description='integration time for <b>explore mode</b>.')
        self.settings.New('explore_mode', bool, initial=False,
                          description='''continuous readout using <b>explore_mode_exposure_time</b> 
                                         and <b>acq_mode</b>=<i>single</i>. Does <b>NOT</b> save data. 
                                         Interruption restores previous camera settings.''')
        self.settings.explore_mode.add_listener(self.set_explore_mode)
        
        self.add_operation('run_acquire_bg', self.acquire_bg_start)
        self.add_operation('run_acquire_single', self.acquire_single_start)
        
        self.settings.New('show_line', bool, initial=False)

        
        
#     def pixel2wavelength(self, grating_position, pixel_index):
#         offset = self.settings['calib_offset'] # nm
#         focal_length = self.settings['calib_focal_length'] # mm
#         delta = self.settings['calib_delta'] # rad
#         gamma = self.settings['calib_gamma'] #rad
#         grating_spacing = 1./self.settings['calib_grating_groves']  #mm
#         pixel_size = self.settings['calib_pixel_size']*1e-3  #mm   #Binning!
#         m_order = self.settings['calib_m_order'] #diffraction order
#         
#         ccd_hw = self.app.hardware['andor_ccd']
#         binning_yx = ccd_hw.settings['ccd_shape']/ ccd_hw.settings['readout_shape']
#         binning = binning_yx[1]
#             
#         wl_center = (grating_position + offset)*1e-6
#         px_from_center = pixel_index*binning +binning/2. - 0.5*ccd_hw.settings['ccd_shape'][1]
#         
#         psi = np.arcsin(m_order* wl_center / (2*grating_spacing*np.cos(gamma/2)))
#         
#         eta = np.arctan(px_from_center*pixel_size*np.cos(delta) /
#         (focal_length+px_from_center*pixel_size*np.sin(delta)))
#         
#         return 1e6*((grating_spacing/m_order)
#                         *(np.sin(psi-0.5*gamma)
#                           + np.sin(psi+0.5*gamma+eta)))        
    
    def acquire_bg_start(self):
        self.acquire_bg.update_value(True)
        self.start()
    
    def acquire_single_start(self):
        '''Deprecated, use continuous settings instead'''
        self.settings.continuous.update_value(False)
        self.start()

        

    def setup_figure(self):

        #if hasattr(self, 'graph_layout'):
        #    self.graph_layout.deleteLater() # see http://stackoverflow.com/questions/9899409/pyside-removing-a-widget-from-a-layout
        #    del self.graph_layout
            
        ui = self.ui = load_qt_ui_file(sibling_path(__file__, 'andor_ccd_readout.ui'))
        
        ## ui connection
        self.hw = andor = self.app.hardware['andor_ccd']
        andor.settings.exposure_time.connect_to_widget(ui.andor_ccd_int_time_doubleSpinBox)
        andor.settings.em_gain.connect_to_widget(ui.andor_ccd_emgain_doubleSpinBox)
        andor.settings.temperature.connect_to_widget(ui.andor_ccd_temp_doubleSpinBox)
        andor.settings.ccd_status.connect_to_widget(ui.andor_ccd_status_label)
        andor.settings.shutter_open.connect_to_widget(ui.andor_ccd_shutter_open_checkBox)
        
        self.settings.continuous.connect_to_widget(ui.andor_ccd_continuous_checkBox)
        self.settings.explore_mode.connect_to_pushButton(ui.run_explore_mode_pushButton, 
                                                         colors=['yellow', 'orange'], 
                                                         texts=['run explore mode', 'interrupt explore mode'])
        self.settings.explore_mode_exposure_time.connect_to_widget(ui.explore_mode_exposure_time_doubleSpinBox)
        
        self.settings.bg_subtract.connect_to_widget(ui.andor_ccd_bgsub_checkBox)
        ui.andor_ccd_acq_bg_pushButton.clicked.connect(self.acquire_bg_start)
        self.settings.activation.connect_to_pushButton(ui.andor_ccd_start_pushButton)
        #ui.andor_ccd_start_pushButton.clicked.connect(self.start)
        #ui.andor_ccd_interrupt_pushButton.clicked.connect(self.interrupt)
        
        andor.settings.temp_status.connect_to_widget(self.ui.temp_status_label)
        andor.settings.temp_setpoint.connect_to_widget(self.ui.temp_setpoint_doubleSpinBox)
        
        self.settings.save_h5.connect_to_widget(self.ui.save_h5_checkBox)
        
        andor.settings.connected.connect_to_widget(self.ui.hw_connect_checkBox)
        
        self.settings.wl_calib.connect_to_widget(self.ui.wl_calib_comboBox)


        #### PLot window
        # NOTE, view toggling is handled in andor_ccd_readout.ui file!!
        self.graph_layout = pg.GraphicsLayoutWidget()
        self.ui.plot_groupBox.layout().addWidget(self.graph_layout)
        self.spec_plot = self.graph_layout.addPlot()
        self.spec_plot_line = self.spec_plot.plot([1,3,2,4,3,5])
        self.spec_plot.enableAutoRange()
        
        ## measure_line
        self.spec_infline = pg.InfiniteLine(movable=True, angle=90, label='x={value:0.2f}', 
                       labelOpts={'position':0.1, 'color': (200,200,100), 'fill': (200,200,200,50), 'movable': True})
        self.settings.show_line.add_listener(self.on_change_show_line)
        self.settings.show_line.connect_to_widget(self.ui.show_spec_line_checkBox)
        
        #### Image window        
        self.img_layout = pg.GraphicsLayoutWidget()
        self.ui.image_groupBox.layout().addWidget(self.img_layout)
        self.img_plot = self.img_layout.addPlot()
        #self.img_plot.getViewBox().setLimits(minXRange=-10, maxXRange=100, minYRange=-10, maxYRange=100)
        self.img_plot.showGrid(x=True, y=True)
        self.img_plot.setAspectLocked(lock=True, ratio=1)
        self.img_item = pg.ImageItem()
        self.img_plot.addItem(self.img_item)
        self.hist_lut = pg.HistogramLUTItem()
        self.hist_lut.autoHistogramRange()
        self.hist_lut.setImageItem(self.img_item)
        self.img_layout.addItem(self.hist_lut)
        self.ui.image_view_checkBox.setCheckState(False) #hide first.
                
        ### CCD settings
        self.cam_controls = self.app.hardware['andor_ccd'].settings.New_UI(style='scroll_form')
        self.ui.ccd_settings_GroupBox.layout().addWidget(self.cam_controls)
        self.ui.show_ccd_settings_checkBox.setCheckState(False) #hide first.
        
        
    def on_change_show_line(self):
        if self.settings['show_line']:
            self.spec_plot.addItem(self.spec_infline)
        else:
            self.spec_plot.removeItem(self.spec_infline)

    def run(self):

        ccd_hw = self.app.hardware['andor_ccd']
        ccd_dev = ccd_hw.ccd_dev
        
        width_px = ccd_dev.Nx_ro
        height_px = ccd_dev.Ny_ro
        
        ccd_hw.settings['trigger_mode'] = 'internal'
        
        t_acq = self.app.hardware['andor_ccd'].settings['exposure_time'] #in seconds
        
        wait_time = 0.01 #np.min(1.0,np.max(0.05*t_acq, 0.05)) # limit update period to 50ms (in ms) or as slow as 1sec
        
        # print('andor_ccd_readout run')
            
        try:
            self.log.info("starting acq")
            
            # print("starting acq")
            ccd_dev.start_acquisition()
        
            self.log.info( "checking..." )
            t0 = time.time()

#             if 'acton_spectrometer' in self.app.hardware and self.app.hardware['acton_spectrometer'].settings['connected']:
#                 self.wls  = self.pixel2wavelength(
#                               self.app.hardware['acton_spectrometer'].settings['center_wl'], 
#                               np.arange(width_px))
#                               #, binning=ccd_dev.get_current_hbin())
#             else:
#                 self.wls = np.arange(width_px)


            while not self.interrupt_measurement_called:

                wl_calib = self.settings['wl_calib']
                hbin = ccd_dev.get_current_hbin()
                if wl_calib=='acton_spectrometer':
                    px_index = np.arange(width_px)
                    spec_hw = self.app.hardware['acton_spectrometer']
                    self.wls = spec_hw.get_wl_calibration(px_index, hbin)
                elif wl_calib=='andor_spectrometer':
                    px_index = np.arange(width_px)
                    spec_hw = self.app.hardware['andor_spec']
                    self.wls = spec_hw.get_wl_calibration(px_index, hbin)   
                elif wl_calib=='pixels':
                    binning = hbin
                    px_index = np.arange(width_px)
                    self.wls = binned_px = binning*px_index + 0.5*(binning-1)
                elif wl_calib=='raw_pixels':
                    self.wls = np.arange(width_px)
                else:
                    self.wls = np.arange(width_px)

            
                stat = ccd_hw.settings.ccd_status.read_from_hardware()
                if stat == 'IDLE':
                    # grab data
                    self.buffer_ = ccd_hw.get_acquired_data()
                                        
                    #print('andor_ccd buffer', self.buffer_.shape, ccd_dev.buffer.shape)
                            
                    if ccd_hw.settings['acq_mode'] == 'accumulate':
                        self.buffer_ = self.buffer_ / ccd_hw.settings['num_acc']
                        print(self.name, 'buffer division', self.buffer_[0][0])
                                        
                    if self.bg_subtract.val and not self.acquire_bg.val:
                        bg = ccd_hw.background
                        if bg is not None:
                            if bg.shape == self.buffer_.shape:
                                self.buffer_ = self.buffer_ - bg
                            else:
                                self.log.warning("Background not the correct shape {} != {}".format( self.buffer_.shape, bg.shape))
                        else:
                            self.log.warning( "No Background available, raw data shown")

                    #print(self.name, 'after bg', self.buffer_[0][0])

                    self.spectra_data = np.average(self.buffer_, axis=0)
                    self.settings['count_rate'] = np.sum(self.spectra_data)/t_acq
 
                    if self.acquire_bg.val or not self.settings.continuous.val:
                        break # end the while loop for non-continuous scans
                    else:
                        # restart acq
                        ccd_dev.start_acquisition()

                    if t_acq > 0.1:
                        self.set_progress(0)
                        t0 = time.time()

                    
                else:
                    #sleep(wait_time)
                    #print("GetTotalNumberImagesAcquired", ccd_dev.get_total_number_images_acquired())
                    #print("get_number_new_images", ccd_dev.get_number_new_images())
                    #print("get_number_available_images", ccd_dev.get_number_available_images())


                    if t_acq > 0.1:    
                        if ccd_hw.settings['acq_mode'] == 'accumulate':
                            pct = 100 * (time.time()-t0)/(t_acq * ccd_hw.settings['num_acc'])
                        else:
                            pct = 100 * (time.time()-t0)/t_acq
                        self.set_progress(pct)

                    sleep(0.01)
                    
                    
                    try:
                        ccd_hw.settings.temperature.read_from_hardware()
                        ccd_hw.settings.temp_status.read_from_hardware()
                    except Exception as err:
                        pass # sometimes temperature can't be read during acquisition
        #except Exception as err:
        #    self.log.error( "{} error: {}".format(self.name, err))
        finally:            
            # while-loop is complete
            ccd_hw.interrupt_acquisition()

            
            #is this right place to put this?
            # Signal emission from other threads ok?
            #self.measurement_state_changed.emit(False)
        
            if self.acquire_bg.val:
                if self.interrupt_measurement_called:
                    ccd_hw.background = None
                else:
                    ccd_hw.background = self.buffer_.copy()
                self.acquire_bg.update_value(False)    
        
            if not self.settings.continuous.val:
                if self.interrupt_measurement_called:
                    self.spectrum = None
                else:
                    self.spectrum = self.buffer_.copy()
                    
                # Save data file
                if self.settings['save_h5']:
                    self.t0 = time.time()
                    self.h5_file = h5_io.h5_base_file(self.app, measurement=self )
                    self.h5_file.attrs['time_id'] = self.t0
                    H = self.h5_meas_group  =  h5_io.h5_create_measurement_group(self, self.h5_file)
                
                    #create h5 data arrays
                    H['wls'] = self.wls
                    H['spectrum'] = self.spectrum
                
                    self.h5_file.close()

                # NPZ data file
                if False: 
                    save_dict = {
                             'spectrum': self.spectrum,
                             'wls': self.wls,
                                }               
                    
                    for lqname,lq in self.app.settings.as_dict().items():
                        save_dict[lqname] = lq.val
                    for hw in self.app.hardware.values():
                        for lqname,lq in hw.settings.as_dict().items():
                            save_dict[hw.name + "_" + lqname] = lq.val
                    for lqname,lq in self.settings.as_dict().items():
                        save_dict[self.name +"_"+ lqname] = lq.val
    
                    self.fname = "%i_%s.npz" % (time.time(), self.name)
                    np.savez_compressed(self.fname, **save_dict)
                    self.log.info( "saved: " + self.fname)
                    
                self.log.info( "Andor CCD single acq successfully acquired")
                # print("Andor CCD single acq successfully acquired")
                # self.settings.continuous.update_value(True)
                
            ccd_hw.settings.ccd_status.read_from_hardware()
            ccd_hw.settings.temperature.read_from_hardware()
            ccd_hw.settings.temp_status.read_from_hardware()

    def update_display(self):
        if hasattr(self, 'buffer_'):
            #print('update_display', self.buffer_.shape)
            if len(self.buffer_.shape) == 2:
                self.img_item.setImage(self.buffer_.astype(np.float32).T, autoLevels=False)
                self.hist_lut.imageChanged(autoLevel=True, autoRange=True)
                y = self.spectra_data
                
            else: # kinetic
                self.img_item.setImage(self.buffer_[:,:,:].sum(axis=0).astype(np.float32).T, autoLevels=False)
                self.hist_lut.imageChanged(autoLevel=True, autoRange=True)
                y = self.buffer_[:,:,:].sum(axis=(0,1))

            x = self.wls        
            
            self.spec_plot_line.setData(x,y)
    
    def get_spectrum(self):
        return np.squeeze(self.spectrum)
    
    def get_wavelengths(self):
        return self.wls
    
    def set_explore_mode(self):
        self.interrupt()
        time.sleep(0.1)
        if self.settings['explore_mode']:
            if not self.hw.settings['connected']: 
                self.hw.settings['connected'] = True
                self.hw.read_from_hardware()
            time.sleep(0.1)
            # store hw and measurement settings
            self.ccd_state0 = {}
            for lqname, lq in self.hw.settings.as_dict().items():
                self.ccd_state0.update({lqname:lq.val})                
            self.continuous0 = self.settings['continuous']
            self.activation0 = self.settings['activation']
            self.save_h50 = self.settings['save_h5']            
            # set fast readout settings
            self.hw.settings['exposure_time'] = self.settings['explore_mode_exposure_time']
            self.hw.settings['acq_mode'] = 'single'            
            self.settings['save_h5'] = False
            self.settings['continuous'] = True
            self.settings['activation'] = True
        else:
            # set to previous (stored) settings
            if hasattr(self, 'ccd_state0'):
                for lqname, val in self.ccd_state0.items():
                    if lqname in ['connected']: #exclude some settings 
                        continue
                    else:
                        self.hw.settings[lqname] = val      
                self.settings['continuous'] = self.continuous0           
                self.settings['save_h5'] = self.save_h50
                self.settings['activation'] = self.activation0           

                  
                  
class AndorCCDStepAndGlue(Measurement):

    name = "andor_ccd_step_and_glue"
    
    def setup(self):
        
        self.display_update_period = 0.050 #seconds

        
        #local logged quantities
        self.bg_subtract = self.add_logged_quantity('bg_subtract', dtype=bool, initial=False, ro=False)
        

        self.center_wl_start = self.add_logged_quantity('center_wl_start', dtype=float, initial=400, ro=False)
        self.center_wl_stop  = self.add_logged_quantity('center_wl_stop', dtype=float, initial=1100, ro=False)
        self.center_wl_step = self.add_logged_quantity('center_wl_step', dtype=float, initial=50, ro=False)

    
    def setup_figure(self):
        ui = self.ui = load_qt_ui_file(sibling_path(__file__, 'andor_ccd_readout.ui'))
    
        #connect events
        self.bg_subtract.connect_bidir_to_widget(self.ui.andor_ccd_bgsub_checkBox)
        
    

    def _run(self):

        # Hardware
        ccd_hw = self.app.hardware['andor_ccd']
        ccd = ccd_hw.ccd_dev
        
        if 'acton_spec' in self.app.hardware:
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
            bg = self.bg = ccd_hw.background
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
            ccd_hw.interrupt_acquisition()
            self.h5_file.close()            
            
    def get_spectrum(self):
        return np.squeeze(self.spectrum)
    
    def get_wavelengths(self):
        return self.wls