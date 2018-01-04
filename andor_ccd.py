'''
Created on May 6, 2014

@author: lab
'''
from __future__ import absolute_import, print_function
from ScopeFoundry import HardwareComponent
from collections import OrderedDict
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
        self.status = self.add_logged_quantity(name='ccd_status', dtype=str, initial="?", fmt="%s",ro=True)
        
        self.temperature = self.add_logged_quantity(name="temperature", dtype=int,
                                                    ro=True, unit = "C", vmin = -300, vmax = 300, si=False)
        
        self.settings.New('temp_setpoint', dtype=int, unit="C", vmin = -300, vmax = 300, si=False)
        self.settings.New('temp_status', dtype=str, ro=True, initial="?",)
        
        self.cooler_on = self.add_logged_quantity(name="cooler_on", dtype=bool, ro=False, initial=True)

        self.exposure_time = self.add_logged_quantity(name="exposure_time", 
                                                      dtype=float,spinbox_decimals=4,
                                                      fmt="%e", ro=False,
                                                      unit = "sec", si=True,
                                                      vmin = 1e-3, vmax=1000)
        
        self.settings.New("has_em_ccd", dtype=bool, ro=True, initial=False)  
        
        self.em_gain = self.add_logged_quantity("em_gain", dtype=int, ro=False,
                                                si=False,
                                                vmin=1, vmax=4096)
        
        self.acq_mode = self.add_logged_quantity('acq_mode', dtype=str, 
                                 initial='single', choices=('single', 'accumulate', 'kinetic', 'run_till_abort') )
        
        
        self.acc_time = self.add_logged_quantity('acc_time', dtype=float, unit='s', initial=0.1, si=True)
        self.kin_time = self.add_logged_quantity('kin_time', dtype=float, unit='s', initial=0.1, si=True)
        self.num_acc = self.add_logged_quantity('num_acc', dtype=int, initial=1, vmin=1)
        self.num_kin = self.add_logged_quantity('num_kin', dtype=int, initial=1, vmin=1)
        
        
        # Output amplifier ( EMCCD or conventional)
        self.output_amp = self.add_logged_quantity("output_amp", dtype=int, ro=False,
                                                   choices = [
                                                              ("0: EMCCD / Default", 0), 
                                                              ("1: Conventional", 1)]
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
        
        self.trigger_mode = self.add_logged_quantity("trigger_mode", dtype=str, initial='internal',
                                                     choices= ("internal","external","external_start",
                                                               "external_exposure","external_fvb_em",
                                                               "software")
                                                     )
        
        # Readout mode
        self.readout_mode = self.add_logged_quantity(name="readout_mode", dtype=str, ro=False,
                                                     initial = 'Image',
                                                     choices = ("Image", "FullVerticalBinning", "SingleTrack",)
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
        
        self.settings.New('ccd_shape', dtype=int, array=True, ro=True)        
        self.settings.New('readout_shape', dtype=int, array=True, ro=True)
        
        # Horizontal and vertical flipping
        self.hflip = self.add_logged_quantity("hflip", 
                                              dtype=bool, initial=True)
        self.vflip = self.add_logged_quantity("vflip", 
                                              dtype=bool, initial=False)

        # A single operation to update the ROI values in the camera
        self.add_operation("set_readout", self.set_readout)
        self.add_operation("set_full_image", self.set_full_image)
        
        
    def connect(self):
        if self.debug: self.log.debug( "Connecting to Andor EMCCD " )
        
        # Open connection to hardware
        self.ccd_dev = AndorCCD(debug = self.debug, initialize_to_defaults=False)

        # connect logged quantities
        self.status.hardware_read_func = self.ccd_dev.get_status
        
        
        ### Temperature
        self.temperature.hardware_read_func = self.ccd_dev.get_temperature
        self.settings.temp_setpoint.connect_to_hardware(write_func=self.ccd_dev.set_temperature)
        self.settings.temp_setpoint.write_to_hardware()
        self.settings.temp_status.connect_to_hardware(self.ccd_dev.get_temperature_status)
        self.cooler_on.connect_to_hardware(write_func=self.ccd_dev.set_cooler)
        self.cooler_on.write_to_hardware()
        
        self.exposure_time.hardware_set_func = self.ccd_dev.set_exposure_time
        self.exposure_time.hardware_read_func = self.ccd_dev.get_exposure_time
        self.exposure_time.write_to_hardware()
        
        self.settings['has_em_ccd'] = self.ccd_dev.has_em_ccd()
        if self.settings['has_em_ccd']:
            self.em_gain.hardware_read_func = self.ccd_dev.get_EMCCD_gain
            self.em_gain.hardware_set_func = self.ccd_dev.set_EMCCD_gain
            self.em_gain.write_to_hardware()
        else:
            self.em_gain.change_readonly(True)
        
        
        self.output_amp.hardware_set_func = self.ccd_dev.set_output_amp
        self.output_amp.write_to_hardware()
        
        self.ad_chan.hardware_set_func = self.ccd_dev.set_ad_channel
        self.ad_chan.write_to_hardware()
        
        if self.settings['has_em_ccd']:
            self.hs_speed_em.hardware_set_func = self.ccd_dev.set_hs_speed_em
        else:
            self.hs_speed_em.change_readonly(True)
        self.vs_speed.hardware_set_func = self.ccd_dev.set_vs_speed
        self.hs_speed_conventional.hardware_set_func = self.ccd_dev.set_hs_speed_conventional
        self.shutter_open.hardware_set_func  = self.ccd_dev.set_shutter_open
        self.shutter_open.write_to_hardware()
        self.trigger_mode.hardware_set_func = self.ccd_dev.set_trigger_mode
        self.trigger_mode.write_to_hardware()
        
        #self.hflip.hardware_read_func = self.ccd_dev.get_image_hflip
        self.hflip.hardware_set_func = self.ccd_dev.set_image_hflip
        self.hflip.write_to_hardware()
        #self.vflip.hardware_read_func = self.ccd_dev.get_image_vflip
        self.vflip.hardware_set_func = self.ccd_dev.set_image_vflip
        self.vflip.write_to_hardware()
        
        
        self.acq_mode.connect_to_hardware(
            #read_func=self.ccd_dev.get_aq_mode,
            write_func=self.ccd_dev.set_aq_mode)
        self.acq_mode.write_to_hardware()
        
        self.num_acc.connect_to_hardware(
            #read_func=self.ccd_dev.get_num_accumulations,
            write_func=self.ccd_dev.set_num_accumulations)
        self.num_acc.write_to_hardware()
        
        self.num_kin.connect_to_hardware(
            #read_func=self.ccd_dev.get_num_kinetics,
            write_func=self.ccd_dev.set_num_kinetics)
        try:
            self.num_kin.write_to_hardware()
        except Exception as err:
            self.log.error("set_num_kinetics failed {}".format(err))
            
        self.acc_time.connect_to_hardware(
            write_func=self.ccd_dev.set_accumulation_cycle_time)
        try:
            self.acc_time.write_to_hardware()
        except Exception as err:
            self.log.error("set_accumulation_cycle_time failed {}".format(err))
            
        self.kin_time.connect_to_hardware(
            write_func=self.ccd_dev.set_kinetic_cycle_time)
        self.kin_time.write_to_hardware()
        
        # Update the ROI min and max values to the CCD dimensions
        width, height = self.ccd_dev.get_detector_shape()        
        self.settings['ccd_shape'] = height, width
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
        
        
        
        # Choices for the horizontal shift speeds in EMCCD mode
        if self.settings['has_em_ccd']:
#             choices = []
#             for chan_i in range(self.ccd_dev.numADChan):
#                 for speed in enumerate(self.ccd_dev.HSSpeeds_EM[chan_i]):
#                     choices.append((
#                                     str.format("Chan {} - {:.2f} MHz", chan_i, speed[1]),
#                                     speed[0]))
#             self.hs_speed_em.change_choice_list(choices)
            
            shift_speed_names = OrderedDict()
            for chan_i in range(self.ccd_dev.numADChan):
                for speed_i, speed in enumerate(self.ccd_dev.HSSpeeds_EM[chan_i]):
                    shift_speed_names[speed_i] = shift_speed_names.get(speed_i, "") + " AD{}-{:.2f}MHz".format(chan_i, speed)
            choices = [ (name, num) for num, name in shift_speed_names.items() ]         
            self.hs_speed_em.change_choice_list(choices)
        
        # Choices for the horizontal shift speeds in conventional mode
        choices = []
#         for chan_i in range(self.ccd_dev.numADChan):
#             for speed in enumerate(self.ccd_dev.HSSpeeds_Conventional[chan_i]):
#                 choices.append((
#                                 str.format("Chan {} - {:.2f} MHz", chan_i,speed[1]),
#                                 speed[0]))
#         self.hs_speed_conventional.change_choice_list(choices)
        shift_speed_names = OrderedDict()
        for chan_i in range(self.ccd_dev.numADChan):
                for speed_i, speed in enumerate(self.ccd_dev.HSSpeeds_Conventional[chan_i]):
                    shift_speed_names[speed_i] = shift_speed_names.get(speed_i, "") + " AD{}-{:.2f}MHz".format(chan_i, speed)
        choices = [ (name, num) for num, name in shift_speed_names.items() ]         
        self.hs_speed_conventional.change_choice_list(choices)
        
        
        # Choices for the vertical shift speeds in conventional mode
        choices = []
        for speed_i in range(self.ccd_dev.numVSSpeeds):    
            choices.append((
                            str.format("Speed {} - {:.2f} us", speed_i, self.ccd_dev.VSSpeeds[speed_i]),
                            speed_i))
        self.vs_speed.change_choice_list(choices)
        
        # Choices for the AD channels
        choices = []
        #chan = self.ad_chan.value
        #print("ad_chan a", chan)
        for chan_i in range(self.ccd_dev.numADChan):
            choices.append((str.format("AD{}", chan_i), chan_i))
        self.ad_chan.change_choice_list(choices)
#         print("ad_chan b", self.ad_chan.value)
#         self.ad_chan.update_value(chan)
#         print("ad_chan c", self.ad_chan.value)
#         self.ad_chan.send_display_updates(force=True)
        
        # For all of the logged quantities, call read from hardware to make sync
        # everything.
        self.read_from_hardware()        
        
        
        
        
        
        # Set some default values that are useful
        #self.ccd_dev.set_temperature(DEFAULT_TEMPERATURE)
        
        
#         if not self.has_been_connected_once:
#             # initialize the readout parameters
#             self.output_amp.update_value(0)        #EMCCD mode
#             self.ad_chan.update_value(0)           #14-bit AD Chan
#             self.hs_speed_em.update_value(0)       #10 MHz readout speed
#             self.vs_speed.update_value(self.ccd_dev.numVSSpeeds-1)          #Slowest vertical shift speed
#             self.hflip.update_value(True)          #Default to true horizontal flip
#             self.exposure_time.update_value(1)     #Default to a 1 s integration
#             self.shutter_open.update_value(False)  #Close the shutter.
#             self.em_gain.update_value(10)
#             self.cooler_on.update_value(True)
#             
#             self.acq_mode.update_value('single')
#         
#             # Readout and ROI parameters
#             self.readout_mode.update_value(AndorReadMode.Image.value)  #Full image readout mode
#             self.roi_img_hstart.update_value(1)
#             self.roi_img_hend.update_value(width)
#             self.roi_img_hbin.update_value(1)
#             self.roi_img_vstart.update_value(1)
#             self.roi_img_vend.update_value(height)
#             self.roi_img_vbin.update_value(1)
#             self.roi_st_center.update_value(height/2)
#             self.roi_st_width.update_value(height/10)
#             self.roi_st_hbin.update_value(1)
#             self.roi_fvb_hbin.update_value(1)
        
        self.set_readout()
        
        #self.is_connected = True
        

    def disconnect(self):
        
        #disconnect logged quantities from hardware
        self.settings.disconnect_all_from_hardware()

        # clean up hardware object

        if hasattr(self, 'ccd_dev'):        
            #disconnect hardware
            self.ccd_dev.close()
    
            
            del self.ccd_dev
        
        self.is_connected = False
    
    def is_background_valid(self):
        bg = self.background
        if bg is not None:
            if bg.shape == self.ccd_dev.buffer.shape:
                return True
            else:
                self.log.debug( "Background not the correct shape {} {}".format(self.ccd_dev.buffer.shape, bg.shape))
        else:
            self.log.info( "No Background available, raw data shown" )
    
        return False
    
    def interrupt_acquisition(self):
        '''If the camera status is not IDLE, calls abort_acquisition()
        '''
        #stat = self.ccd_dev.get_status()
        stat = self.settings.ccd_status.read_from_hardware()
        if stat != 'IDLE':
            self.ccd_dev.abort_acquisition()
        stat = self.settings.ccd_status.read_from_hardware()
        
    
    def set_readout(self):
        """Sets ROI based on values in LoggedQuantities for the current readout mode
        Also sets the flip"""
        
        self.ccd_dev.set_image_flip(self.hflip.val, self.vflip.val)
        
        ro_mode = self.readout_mode.val
        #ro_mode = self.ccd_dev.get_ro_mode # FIXME
        if ro_mode ==  'FullVerticalBinning':
            self.ccd_dev.set_ro_full_vertical_binning(self.roi_fvb_hbin.val)
        elif ro_mode ==  'Image':
            self.ccd_dev.set_ro_image_mode(
                                             self.roi_img_hbin.val,
                                             self.roi_img_vbin.val, 
                                             self.roi_img_hstart.val,
                                             self.roi_img_hend.val,
                                             self.roi_img_vstart.val,
                                             self.roi_img_vend.val)
        elif ro_mode ==  'SingleTrack':
            self.ccd_dev.set_ro_single_track(self.roi_st_center.val, self.roi_st_width.val, self.roi_st_hbin.val)
        else:
            raise NotImplementedError("ro mode not implemented %s", ro_mode)
        
        self.settings['readout_shape'] = [self.ccd_dev.Ny_ro, self.ccd_dev.Nx_ro]
    
    def read_temp_op(self):
        #print self.ccd_dev.get_status()

        self.log.debug("get_temperature_range: {}".format(self.ccd_dev.get_temperature_range()))
        self.log.debug("get_temperature: {}".format(self.ccd_dev.get_temperature()))
        self.log.debug("get_cooler: {}".format(self.ccd_dev.get_cooler()))
        #self.gui.ui.andor_ccd_shutter_open_checkBox.setChecked(True)
    
    
    def set_full_image(self):
        width, height = self.ccd_dev.get_detector_shape()        
        
        # Readout and ROI parameters
        self.readout_mode.update_value('Image')  #Full image readout mode
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
        
        
    def get_acquired_data(self):
        
        buffer_ = self.ccd_dev.get_acquired_data()
                    
        # If using second output amplifier
        #image will be flipped horizontally
        # so we correct this here
        if self.settings['output_amp'] == 1:
            buffer_ = buffer_[:,::-1]
            
        return buffer_