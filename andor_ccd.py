'''
Created on May 6, 2014

@author: lab
'''
from __future__ import absolute_import, print_function
from ScopeFoundry import HardwareComponent
try:
    from .andor_ccd_interface import AndorCCD, AndorReadMode, DEFAULT_TEMPERATURE
except Exception as err:
    print("Could not load modules needed for AndorCCD:", err)

class AndorCCDHW(HardwareComponent):
    
    def setup(self):
        self.name = "andor_ccd"
        self.debug = True
        self.background = None
        
        # Create logged quantities
        self.status = self.add_logged_quantity(name='ccd_satus', dtype=str, fmt="%s",ro=True)
        
        self.temperature = self.add_logged_quantity(name="temperature", dtype=int,
                                                    ro=True, unit = "C", vmin = -300, vmax = 300, si=False)
        
        self.cooler_on = self.add_logged_quantity(name="cooler_on", dtype=bool, ro=False)

        self.exposure_time = self.add_logged_quantity(name="exposure_time", 
                                                      dtype=float,
                                                      fmt="%e", ro=False,
                                                      unit = "sec", si=True,
                                                      vmin = 1e-3, vmax=1000)     
        
        self.em_gain = self.add_logged_quantity("em_gain", dtype=int, ro=False,
                                                si=False,
                                                vmin=1, vmax=4096)
        
        # Ouput amplifer ( EMCCD or conventional)
        self.output_amp = self.add_logged_quantity("output_amp", dtype=int, ro=False,
                                                   choices = [
                                                              ("EMCCD", 0), 
                                                              ("Conventional", 1)]
                                                   )
        
        #AD Channel and horizontal shift speeds for EM and Conventional modes.
        #There are empty until connected; then filled in with camera-specific values
        self.ad_chan = self.add_logged_quantity("ad_chan", dtype=int, 
                                                 choices=[('', 0)], initial=0)
        self.hs_speed_em = self.add_logged_quantity("hs_speed_em", dtype=int, 
                                                    choices=[('', 0)], initial=0)
        self.hs_speed_conventional = self.add_logged_quantity("hs_chan_conventional", 
                                                              dtype=int, choices=[('', 0)], initial=0)
        
        self.vs_speed = self.add_logged_quantity("vertical_shift_speed",
                                                dtype=int, choices=[('', 0)], initial=0 )
        
        self.shutter_open = self.add_logged_quantity("shutter_open", dtype=bool, 
                                                     ro=False, initial=False)
        
        self.trigger_mode = self.add_logged_quantity("trigger_mode", dtype=str,
                                                     choices=[
                                                         ("internal","internal"),                                                    
                                                         ("external","external"),
                                                         ("external_start","external_start"),
                                                         ("external_exposure","external_exposure"),
                                                         ("external_fvb_em","external_fvb_em"),
                                                         ("software","software")]
                                                     )
        
        # Readout mode
        self.readout_mode = self.add_logged_quantity(name="readout_mode", dtype=int, ro=False,
                                                     initial = 0,
                                                     choices=[('',0)],
                                                              )
        
        # ROI Parameters for image readout mode.
        self.roi_img_hstart = self.add_logged_quantity("roi_img_hstart", dtype=int, unit='px', 
                                                        ro=False, initial=1)
        self.roi_img_hend = self.add_logged_quantity("roi_img_hend", dtype=int, unit='px', 
                                                     ro=False, initial=512)
        self.roi_img_hbin = self.add_logged_quantity("roi_img_hbin", dtype=int, unit='px', 
                                                     ro=False, initial=1)
        self.roi_img_vstart = self.add_logged_quantity("roi_img_vstart", dtype=int, unit='px', 
                                                       initial=1, ro=False)
        self.roi_img_vend = self.add_logged_quantity("roi_img_vend", dtype=int, unit='px', 
                                                     initial=512, ro=False)
        self.roi_img_vbin = self.add_logged_quantity("roi_img_vbin", dtype=int, unit='px', 
                                                     initial=1, ro=False)
        
        #ROI Parameters for single track readout mode
        self.roi_st_center = self.add_logged_quantity("roi_st_center", dtype=int, unit='px', 
                                                      ro=False, initial=256)
        self.roi_st_width = self.add_logged_quantity("roi_st_width", dtype=int, unit='px', 
                                                     ro=False, initial=10)
        self.roi_st_hbin = self.add_logged_quantity("roi_st_hbin", dtype=int, unit='px', 
                                                    ro=False, initial=1)
        
        #ROI parameters for full vertical binning
        self.roi_fvb_hbin = self.add_logged_quantity("roi_fvb_hbin", dtype=int, unit='px', 
                                                     ro=False, initial=1)
        
        # Horizontal and vertical flipping
        self.hflip = self.add_logged_quantity("hflip", 
                                              dtype=bool, initial=True)
        self.vflip = self.add_logged_quantity("vflip", 
                                              dtype=bool, initial=False)

        # A single operation to update the ROI values in the camera
        self.add_operation("set_readout", self.set_readout)
        self.add_operation("read_temp", self.read_temp_op)
        
        #connect to custom gui - NOTE:  these are not disconnected!
        if hasattr(self.gui.ui, 'andor_ccd_int_time_doubleSpinBox'):
            self.exposure_time.connect_bidir_to_widget(self.gui.ui.andor_ccd_int_time_doubleSpinBox) 
            #self.gui.ui.andor_ccd_int_time_doubleSpinBox.valueChanged[float].connect(self.exposure_time.update_value)
            self.exposure_time.updated_value[float].connect(self.gui.ui.andor_ccd_int_time_doubleSpinBox.setValue)
            self.temperature.updated_value[float].connect(self.gui.ui.andor_ccd_temp_doubleSpinBox.setValue)
            self.gui.ui.andor_ccd_emgain_doubleSpinBox.valueChanged[float].connect(self.em_gain.update_value)
            self.em_gain.updated_value[float].connect(self.gui.ui.andor_ccd_emgain_doubleSpinBox.setValue)
            self.gui.ui.andor_ccd_shutter_open_checkBox.stateChanged[int].connect(self.shutter_open.update_value)
            self.shutter_open.updated_value[bool].connect(self.gui.ui.andor_ccd_shutter_open_checkBox.setChecked)
            self.status.updated_text_value[str].connect(self.gui.ui.andor_ccd_status_label.setText)
        
    def connect(self):
        if self.debug: self.log.debug( "Connecting to Andor EMCCD Counter" )
        
        # Open connection to hardware
        self.andor_ccd = AndorCCD(debug = self.debug)

        # connect logged quantities
        self.status.hardware_read_func = self.andor_ccd.get_status
        self.temperature.hardware_read_func = self.andor_ccd.get_temperature
        self.exposure_time.hardware_set_func = self.andor_ccd.set_exposure_time
        self.exposure_time.hardware_read_func = self.andor_ccd.get_exposure_time
        self.em_gain.hardware_read_func = self.andor_ccd.get_EMCCD_gain
        self.em_gain.hardware_set_func = self.andor_ccd.set_EMCCD_gain
        self.output_amp.hardware_read_func = self.andor_ccd.get_output_amp
        self.output_amp.hardware_set_func = self.andor_ccd.set_output_amp
        self.ad_chan.hardware_set_func = self.andor_ccd.set_ad_channel
        self.hs_speed_em.hardware_set_func = self.andor_ccd.set_hs_speed_em
        self.vs_speed.hardware_set_func = self.andor_ccd.set_vs_speed
        self.hs_speed_conventional.hardware_set_func = self.andor_ccd.set_hs_speed_conventional
        self.shutter_open.hardware_set_func  = self.andor_ccd.set_shutter_open
        self.trigger_mode.hardware_set_func = self.andor_ccd.set_trigger_mode
        self.hflip.hardware_read_func = self.andor_ccd.get_image_hflip
        self.hflip.hardware_set_func = self.andor_ccd.set_image_hflip
        self.vflip.hardware_read_func = self.andor_ccd.get_image_vflip
        self.vflip.hardware_set_func = self.andor_ccd.set_image_vflip
        
        # Update the ROI min and max values to the CCD dimensions
        width = self.andor_ccd.Nx
        height = self.andor_ccd.Ny
        self.roi_fvb_hbin.change_min_max(1, width)
        self.roi_img_hbin.change_min_max(1, width)
        self.roi_img_hend.change_min_max(1, width)
        self.roi_img_hstart.change_min_max(1, width)
        self.roi_img_vbin.change_min_max(1, height)
        self.roi_img_vend.change_min_max(1, height)
        self.roi_img_vstart.change_min_max(1, height)
        self.roi_st_center.change_min_max(1, height)
        self.roi_st_hbin.change_min_max(1, width)
        self.roi_st_width.change_min_max(1, height)
        
        #Choics for Readout mode
        choices = [("FullVerticalBinning", AndorReadMode.FullVerticalBinning.value),
                      ("SingleTrack", AndorReadMode.SingleTrack.value),
                      ("Image", AndorReadMode.Image.value)
                    ]
        self.readout_mode.change_choice_list(choices)
        
        
        # Choices for the horizontal shift speeds in EMCCD mode
        choices = []
        for chan_i in range(self.andor_ccd.numADChan):
            for speed in enumerate(self.andor_ccd.HSSpeeds_EM[chan_i]):
                choices.append((
                                str.format("Chan {} - {:.2f} MHz", chan_i, speed[1]),
                                speed[0]))
        self.hs_speed_em.change_choice_list(choices)
        
        # Choices for the horizontal shift speeds in conventional mode
        choices = []
        for chan_i in range(self.andor_ccd.numADChan):
            for speed in enumerate(self.andor_ccd.HSSpeeds_Conventional[chan_i]):
                choices.append((
                                str.format("Chan {} - {:.2f} MHz", chan_i,speed[1]),
                                speed[0]))
        self.hs_speed_conventional.change_choice_list(choices)
        
        # Choices for the vertical shift speeds in conventional mode
        choices = []
        for speed_i in range(self.andor_ccd.numVSSpeeds):    
            choices.append((
                            str.format("Speed {} - {:.2f} us", speed_i, self.andor_ccd.VSSpeeds[speed_i]),
                            speed_i))
        self.vs_speed.change_choice_list(choices)
        
        # Choices for the AD channels
        choices = []
        for chan_i in range(self.andor_ccd.numADChan):
            choices.append((str.format("Channel {}", chan_i), chan_i))
        self.ad_chan.change_choice_list(choices)
        
        # For all of the logged quantities, call read from hardware to make sync
        # everything.
        for name, lq in self.logged_quantities.items():
            lq.read_from_hardware()        
        
        # Set some default values that are useful
        self.andor_ccd.set_temperature(DEFAULT_TEMPERATURE)
        
        if not self.has_been_connected_once:
            # initialize the readout parameters
            self.output_amp.update_value(0)        #EMCCD mode
            self.ad_chan.update_value(0)           #14-bit AD Chan
            self.hs_speed_em.update_value(0)       #10 MHz readout speed
            self.vs_speed.update_value(self.andor_ccd.numVSSpeeds-1)          #Slowest vertical shift speed
            self.hflip.update_value(True)          #Default to true horizontal flip
            self.exposure_time.update_value(1)     #Default to a 1 s integration
            self.shutter_open.update_value(False)  #Close the shutter.
            self.em_gain.update_value(10)
            self.cooler_on.update_value(True)
        
            # Readout and ROI parameters
            self.readout_mode.update_value(AndorReadMode.Image.value)  #Full image readout mode
            self.roi_img_hstart.update_value(1)
            self.roi_img_hend.update_value(width)
            self.roi_img_hbin.update_value(1)
            self.roi_img_vstart.update_value(1)
            self.roi_img_vend.update_value(height)
            self.roi_img_vbin.update_value(1)
            self.roi_st_center.update_value(height/2)
            self.roi_st_width.update_value(height/10)
            self.roi_st_hbin.update_value(1)
            self.roi_fvb_hbin.update_value(1)
        
        self.set_readout()
        
        self.is_connected = True
        

    def disconnect(self):
        #disconnect hardware
        self.andor_ccd.close()
        
        #disconnect logged quantities from hardware
        for lq in self.logged_quantities.values():
            lq.hardware_read_func = None
            lq.hardware_set_func = None
        
        # clean up hardware object
        del self.andor_ccd
        
        self.is_connected = False
    
    def is_background_valid(self):
        bg = self.background
        if bg is not None:
            if bg.shape == self.andor_ccd.buffer.shape:
                return True
            else:
                self.log.debug( "Background not the correct shape {} {}".format(self.andor_ccd.buffer.shape, bg.shape))
        else:
            self.log.info( "No Background available, raw data shown" )
    
        return False
    
    def interrupt_acquisition(self):
        '''If the camera status is not IDLE, calls abort_acquisition()
        '''
        stat = self.andor_ccd.get_status()
        if stat != 'IDLE':
            self.andor_ccd.abort_acquisition()
    
    def set_readout(self):
        """Sets ROI based on values in LoggedQuantities for the current readout mode
        Also sets the flip"""
        
        self.andor_ccd.set_image_flip(self.hflip.val, self.vflip.val)
        
        ro_mode = self.readout_mode.val
        #ro_mode = self.andor_ccd.get_ro_mode # FIXME
        if ro_mode ==  AndorReadMode.FullVerticalBinning.value:
            self.andor_ccd.set_ro_full_vertical_binning(self.roi_fvb_hbin.val) #FIXME
        elif ro_mode ==  AndorReadMode.Image.value:
            self.andor_ccd.set_ro_image_mode(
                                             self.roi_img_hbin.val,
                                             self.roi_img_vbin.val, 
                                             self.roi_img_hstart.val,
                                             self.roi_img_hend.val,
                                             self.roi_img_vstart.val,
                                             self.roi_img_vend.val)
        elif ro_mode ==  AndorReadMode.SingleTrack.value:
            self.andor_ccd.set_ro_single_track(self.roi_st_center.val, self.roi_st_width.val, self.roi_st_hbin.val)
        else:
            raise NotImplementedError("ro mode not implemented %s", ro_mode)
    
    def read_temp_op(self):
        #print self.andor_ccd.get_status()

        self.log.debug("get_temperature_range: {}".format(self.andor_ccd.get_temperature_range()))
        self.log.debug("get_temperature: {}".format(self.andor_ccd.get_temperature()))
        self.log.debug("get_cooler: {}".format(self.andor_ccd.get_cooler()))
        #self.gui.ui.andor_ccd_shutter_open_checkBox.setChecked(True)
        
